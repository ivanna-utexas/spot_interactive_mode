"""Bounded ring buffer with minimal locking for audio capture.

Design:
- Pre-allocated numpy array to avoid GC pauses
- Threading lock held only during pointer updates (not during memcpy)
- On overflow: drop oldest frames, increment drop counter
- Exposes fill level and drop count for diagnostics
"""

import threading

import numpy as np


class AudioRingBuffer:
    """Thread-safe bounded ring buffer for interleaved audio frames.

    The buffer stores raw interleaved audio samples as a flat numpy array.
    Writers append frames (callback thread), readers pop frames (worker thread).
    """

    def __init__(self, max_frames: int, num_channels: int, dtype=np.float32):
        """Initialize the ring buffer.

        Args:
            max_frames: Maximum number of audio frames (samples per channel) to store.
            num_channels: Number of interleaved channels per frame.
            dtype: Sample data type (must match sounddevice output).
        """
        self._max_frames = max_frames
        self._num_channels = num_channels
        self._dtype = dtype
        self._frame_size = num_channels  # samples per frame

        # Pre-allocate storage: flat array of (max_frames * num_channels)
        self._buf = np.zeros(max_frames * num_channels, dtype=dtype)
        self._write_pos = 0  # in frames
        self._read_pos = 0   # in frames
        self._frames_stored = 0

        self._lock = threading.Lock()

        # Diagnostics
        self._dropped_frames = 0
        self._total_written = 0

    def write(self, data: np.ndarray) -> int:
        """Write interleaved audio frames into the buffer.

        Called from the sounddevice callback — must be fast.
        Data shape: (num_frames, num_channels) or (num_frames * num_channels,).

        Args:
            data: Interleaved audio data.

        Returns:
            Number of frames actually written (may be less than input if overflow).
        """
        if data.ndim == 2:
            num_frames = data.shape[0]
            flat = data.ravel()
        else:
            num_frames = len(data) // self._num_channels
            flat = data

        if num_frames == 0:
            return 0

        with self._lock:
            available = self._max_frames - self._frames_stored
            if num_frames > available:
                # Overflow: drop oldest frames to make room
                drop_count = num_frames - available
                self._read_pos = (self._read_pos + drop_count) % self._max_frames
                self._frames_stored -= drop_count
                self._dropped_frames += drop_count

            # Write into circular buffer (may wrap)
            write_samples = num_frames * self._frame_size
            write_start = self._write_pos * self._frame_size
            buf_len = len(self._buf)

            if write_start + write_samples <= buf_len:
                self._buf[write_start:write_start + write_samples] = flat[:write_samples]
            else:
                # Wrap around
                first_chunk = buf_len - write_start
                self._buf[write_start:] = flat[:first_chunk]
                self._buf[:write_samples - first_chunk] = flat[first_chunk:write_samples]

            self._write_pos = (self._write_pos + num_frames) % self._max_frames
            self._frames_stored += num_frames
            self._total_written += num_frames

        return num_frames

    def read(self, max_frames: int = 0) -> np.ndarray:
        """Read and consume frames from the buffer.

        Called from the worker thread.

        Args:
            max_frames: Maximum frames to read. 0 = read all available.

        Returns:
            Interleaved audio data as (num_frames, num_channels) numpy array.
            Empty array if no data available.
        """
        with self._lock:
            if self._frames_stored == 0:
                return np.empty((0, self._num_channels), dtype=self._dtype)

            to_read = self._frames_stored if max_frames == 0 else min(max_frames, self._frames_stored)
            read_start = self._read_pos * self._frame_size
            read_samples = to_read * self._frame_size
            buf_len = len(self._buf)

            if read_start + read_samples <= buf_len:
                out = self._buf[read_start:read_start + read_samples].copy()
            else:
                first_chunk = buf_len - read_start
                out = np.concatenate([
                    self._buf[read_start:],
                    self._buf[:read_samples - first_chunk]
                ])

            self._read_pos = (self._read_pos + to_read) % self._max_frames
            self._frames_stored -= to_read

        return out.reshape(-1, self._num_channels)

    @property
    def fill_level(self) -> float:
        """Current fill level as a fraction [0.0, 1.0]."""
        with self._lock:
            return self._frames_stored / self._max_frames if self._max_frames > 0 else 0.0

    @property
    def frames_available(self) -> int:
        """Number of frames available to read."""
        with self._lock:
            return self._frames_stored

    @property
    def dropped_frames(self) -> int:
        """Total number of frames dropped due to overflow."""
        with self._lock:
            return self._dropped_frames

    @property
    def total_written(self) -> int:
        """Total number of frames written since creation."""
        with self._lock:
            return self._total_written

    def reset(self):
        """Reset the buffer state (e.g. on device reconnect)."""
        with self._lock:
            self._write_pos = 0
            self._read_pos = 0
            self._frames_stored = 0
            self._dropped_frames = 0
            self._total_written = 0
            self._buf[:] = 0
