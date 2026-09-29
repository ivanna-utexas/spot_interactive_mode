"""Audio capture node for Azure Kinect microphone array.

Captures multi-channel audio from the Azure Kinect mic array via ALSA/PortAudio,
mixes to mono, resamples to target rate (16 kHz), and publishes AudioData messages.

Architecture:
- sounddevice callback: copies raw interleaved frames into a bounded ring buffer (no processing)
- Worker thread: pops frames, applies mono mix, streams through soxr resampler, publishes

Threading model: This node uses a SingleThreadedExecutor (rclpy default).
The sounddevice callback runs on PortAudio's own thread; all ROS callbacks and
the worker thread are the only other execution contexts.  The worker is the sole
consumer of the ring buffer and timestamp deque.
"""

import collections
import os
import struct
import threading
import time
import wave

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import sounddevice as sd

from spot_audio_msgs.msg import AudioData
from spot_audio.ring_buffer import AudioRingBuffer


# How long the ring buffer can hold before overflow (seconds)
_BUFFER_DURATION_S = 5.0

# Worker thread poll interval when buffer is empty
_POLL_INTERVAL_S = 0.01

# Maximum output buffer duration before dropping oldest samples (seconds)
_OUTPUT_CAP_S = 5.0


class TimestampAccumulator:
    """Drift-free nanosecond timestamp tracking for arbitrary sample rates.

    For sample rates that don't evenly divide 1e9 (e.g. 48000 Hz -> 20833.333 ns/sample),
    naive ``n * 1e9 // sr`` accumulates truncation drift.  This uses a rational
    remainder accumulator (Bresenham-style) to stay exact.
    """

    __slots__ = ('ns', 'rem', 'sr')

    def __init__(self, start_ns: int, sample_rate: int):
        self.ns = start_ns
        self.rem = 0
        self.sr = sample_rate

    def advance(self, n_samples: int) -> int:
        """Advance by *n_samples*.  Returns the new timestamp (ns)."""
        delta = n_samples * 1_000_000_000
        self.ns += delta // self.sr
        self.rem += delta % self.sr
        self.ns += self.rem // self.sr
        self.rem %= self.sr
        return self.ns

    @property
    def current(self) -> int:
        return self.ns

    def copy(self) -> 'TimestampAccumulator':
        c = TimestampAccumulator(self.ns, self.sr)
        c.rem = self.rem
        return c


class AudioCaptureNode(Node):

    def __init__(self):
        super().__init__('audio_capture_node')

        # -- Parameters --
        self.declare_parameter('device_name', 'Azure Kinect')
        self.declare_parameter('device_id', -1)
        self.declare_parameter('target_sample_rate', 16000)
        self.declare_parameter('chunk_duration_ms', 100)
        self.declare_parameter('mix_strategy', 'mean')
        self.declare_parameter('buffer_warn_threshold', 0.8)
        self.declare_parameter('enable_wav_dump', False)
        self.declare_parameter('stall_timeout_s', 5.0)
        self.declare_parameter('jitter_warn_threshold_ms', 50.0)
        self.declare_parameter('gain_db', 30.0)

        self._device_name = self.get_parameter('device_name').value
        self._device_id = self.get_parameter('device_id').value
        self._target_sr = self.get_parameter('target_sample_rate').value
        self._chunk_ms = self.get_parameter('chunk_duration_ms').value
        self._mix_strategy = self.get_parameter('mix_strategy').value
        self._warn_threshold = self.get_parameter('buffer_warn_threshold').value
        self._wav_dump = self.get_parameter('enable_wav_dump').value
        self._stall_timeout_s = self.get_parameter('stall_timeout_s').value
        self._jitter_warn_ms = self.get_parameter('jitter_warn_threshold_ms').value
        gain_db = self.get_parameter('gain_db').value
        self._gain = 10.0 ** (gain_db / 20.0)  # dB to linear
        if gain_db != 0.0:
            self.get_logger().info(f'Applying {gain_db:.1f} dB gain ({self._gain:.1f}x)')

        # Publisher with best_effort QoS for live audio
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self._pub = self.create_publisher(AudioData, '/spot_audio/audio', qos)

        # Discover and configure audio device
        self._native_sr = None
        self._num_channels = None
        self._sd_device_idx = None
        self._discover_device()

        # Ring buffer: holds _BUFFER_DURATION_S of audio at native rate
        buf_frames = int(self._native_sr * _BUFFER_DURATION_S)
        self._ring = AudioRingBuffer(buf_frames, self._num_channels, dtype=np.float32)

        # Streaming resampler (if needed)
        self._resampler = None
        if self._native_sr != self._target_sr:
            try:
                import soxr
                self._resampler = soxr.ResampleStream(
                    self._native_sr, self._target_sr,
                    num_channels=1, dtype=np.float32,
                )
                self.get_logger().info(
                    f'Streaming resampler: {self._native_sr} -> {self._target_sr} Hz (soxr)'
                )
            except ImportError:
                self.get_logger().fatal(
                    'soxr not installed but resampling needed '
                    f'(native={self._native_sr}, target={self._target_sr}). '
                    'Install with: pip3 install soxr'
                )
                raise

        # Output sample tracking
        self._sample_index = 0
        self._chunk_samples = int(self._target_sr * self._chunk_ms / 1000)

        # -- Callback timestamp state --
        # The sounddevice callback records (native_frame_count, block_start_ns) tuples
        # into _ts_deque.  The worker consumes them in lock-step with ring buffer reads
        # to associate timestamps with audio.
        self._ts_deque: collections.deque = collections.deque()
        self._ts_lock = threading.Lock()
        self._last_callback_ns: int = 0  # monotonic ns, updated by callback

        # -- Callback status counting --
        self._status_lock = threading.Lock()
        self._status_counts: dict[str, int] = {}

        # -- Worker timestamp tracking (native domain) --
        self._native_ts_accum: TimestampAccumulator | None = None
        self._pending_ts_remaining: int = 0

        # -- Output buffer: coupled (audio, sample_index, ts_accum) chunks --
        self._output_chunks: collections.deque = collections.deque()
        self._output_samples: int = 0
        self._output_ts_accum: TimestampAccumulator | None = None

        # -- Ring overflow tracking --
        self._last_drop_count: int = 0
        self._drops_ring_overflow: int = 0

        # -- Output cap tracking --
        self._drops_output_cap: int = 0

        # -- Publish continuity tracking --
        self._prev_capture_end_ns: int | None = None
        self._prev_sample_index: int | None = None

        # -- Warning throttle times --
        self._last_warn_time_ring: float = 0.0
        self._last_warn_time_jitter: float = 0.0
        self._last_warn_time_output_cap: float = 0.0
        self._last_warn_time_fill: float = 0.0

        # WAV dump for debugging
        self._wav_file = None
        if self._wav_dump:
            wav_path = '/tmp/spot_audio_debug.wav'
            self._wav_file = wave.open(wav_path, 'wb')
            self._wav_file.setnchannels(1)
            self._wav_file.setsampwidth(2)  # 16-bit
            self._wav_file.setframerate(self._target_sr)
            self.get_logger().info(f'WAV dump enabled: {wav_path}')

        # Start audio stream + worker thread
        self._running = True
        self._stream = None
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._start_stream()

        # Watchdog timer — checks _running flag and stall timeout
        self._watchdog_timer = self.create_timer(1.0, self._watchdog_cb)

        self.get_logger().info('audio_capture_node started')

    # ------------------------------------------------------------------ #
    # Device discovery
    # ------------------------------------------------------------------ #

    def _discover_device(self):
        """Find and validate the audio input device."""
        devices = sd.query_devices()

        selected = None
        selected_idx = None

        if self._device_id >= 0:
            if self._device_id >= len(devices):
                self.get_logger().fatal(
                    f'Device index {self._device_id} out of range '
                    f'(found {len(devices)} devices)'
                )
                raise RuntimeError(f'Invalid device_id: {self._device_id}')
            selected = devices[self._device_id]
            selected_idx = self._device_id
        else:
            candidates = []
            for i, dev in enumerate(devices):
                if (self._device_name.lower() in dev['name'].lower()
                        and dev['max_input_channels'] > 0):
                    candidates.append((i, dev))

            if len(candidates) > 1:
                self.get_logger().error(
                    f'Multiple devices match "{self._device_name}" with input channels:'
                )
                hostapis = sd.query_hostapis()
                for i, dev in candidates:
                    api_name = hostapis[dev['hostapi']]['name'] if 'hostapi' in dev else '?'
                    self.get_logger().error(
                        f'  [{i}] {dev["name"]} '
                        f'(channels={dev["max_input_channels"]}, '
                        f'hostapi={api_name})'
                    )
                self.get_logger().fatal(
                    'Ambiguous device match. Set device_id in audio.yaml or use '
                    'a more specific device_name.'
                )
                raise RuntimeError('Ambiguous device match')

            if candidates:
                selected_idx, selected = candidates[0]

        if selected is None:
            self.get_logger().error('Available audio input devices:')
            hostapis = sd.query_hostapis()
            for i, dev in enumerate(devices):
                if dev['max_input_channels'] > 0:
                    api_name = hostapis[dev['hostapi']]['name'] if 'hostapi' in dev else '?'
                    self.get_logger().error(
                        f'  [{i}] {dev["name"]} '
                        f'(channels={dev["max_input_channels"]}, '
                        f'hostapi={api_name})'
                    )
            self.get_logger().fatal(
                f'Audio device not found (name="{self._device_name}", id={self._device_id})'
            )
            raise RuntimeError('Audio device not found')

        self._sd_device_idx = selected_idx
        self._num_channels = selected['max_input_channels']

        # Probe the actual capture sample rate.  default_samplerate from
        # query_devices() can reflect the playback side on USB devices
        # (e.g. Anker PowerConf reports 48 kHz default but capture is 16 kHz only).
        self._native_sr = self._probe_capture_rate(selected_idx, selected)

        self.get_logger().info(
            f'Audio device selected: [{selected_idx}] "{selected["name"]}" | '
            f'channels={self._num_channels} | '
            f'native_sr={self._native_sr} | '
            f'format=float32'
        )

        if self._num_channels < 1:
            self.get_logger().fatal('Selected device has no input channels')
            raise RuntimeError('No input channels')

    def _probe_capture_rate(self, dev_idx: int, dev: dict) -> int:
        """Determine the sample rate to open the capture stream at.

        USB Audio devices can report a default_samplerate from the playback
        side.  This method probes the device to find a rate that actually
        works for capture, trying in order:
        1. default_samplerate (works for most devices)
        2. target_sample_rate (ideal — avoids resampling)
        3. common rates (48000, 44100, 16000)
        """
        default_sr = int(dev['default_samplerate'])
        probe_rates = []
        # Try default first, then target, then common rates (deduplicated, order-preserving)
        for rate in [default_sr, self._target_sr, 48000, 44100, 16000]:
            if rate not in probe_rates:
                probe_rates.append(rate)

        for rate in probe_rates:
            try:
                sd.check_input_settings(
                    device=dev_idx, channels=1,
                    samplerate=rate, dtype='float32',
                )
                if rate != default_sr:
                    self.get_logger().info(
                        f'Device default_samplerate ({default_sr} Hz) not supported '
                        f'for capture; using {rate} Hz'
                    )
                return rate
            except sd.PortAudioError:
                continue

        self.get_logger().fatal(
            f'No supported capture sample rate found for device [{dev_idx}] '
            f'"{dev["name"]}". Tried: {probe_rates}'
        )
        raise RuntimeError('No supported capture sample rate')

    # ------------------------------------------------------------------ #
    # Audio stream
    # ------------------------------------------------------------------ #

    def _start_stream(self):
        """Open the audio input stream."""
        try:
            self._stream = sd.InputStream(
                device=self._sd_device_idx,
                samplerate=self._native_sr,
                channels=self._num_channels,
                dtype='float32',
                blocksize=0,
                callback=self._audio_callback,
            )
            self._stream.start()
            self._last_callback_ns = time.monotonic_ns()
            self.get_logger().info('Audio stream started')
        except Exception as e:
            self.get_logger().fatal(f'Failed to start audio stream: {e}')
            raise

    def _audio_callback(self, indata, frames, time_info, status):
        """sounddevice callback — copy + timestamp, return immediately.

        indata shape: (frames, num_channels), dtype float32.
        """
        t_cb_ns = time.monotonic_ns()
        # Back-date by block duration to estimate first-sample time (integer math)
        block_dur_ns = frames * 1_000_000_000 // self._native_sr
        block_start_ns = t_cb_ns - block_dur_ns

        if status:
            with self._status_lock:
                s = str(status)
                self._status_counts[s] = self._status_counts.get(s, 0) + 1

        self._last_callback_ns = t_cb_ns
        self._ring.write(indata)

        with self._ts_lock:
            self._ts_deque.append((frames, block_start_ns))

    # ------------------------------------------------------------------ #
    # Mono mix
    # ------------------------------------------------------------------ #

    def _mono_mix(self, data: np.ndarray) -> np.ndarray:
        """Mix multi-channel audio to mono.

        Args:
            data: Shape (num_frames, num_channels), float32.

        Returns:
            Shape (num_frames,), float32, mono signal.
        """
        if data.shape[1] == 1:
            return data[:, 0]

        strategy = self._mix_strategy

        if strategy == 'mean':
            return data.mean(axis=1)
        elif strategy == 'max_energy_channel':
            rms = np.sqrt(np.mean(data ** 2, axis=0))
            best = int(np.argmax(rms))
            return data[:, best]
        elif strategy.startswith('channel:'):
            ch = int(strategy.split(':')[1])
            if ch >= data.shape[1]:
                self.get_logger().warning(
                    f'Channel {ch} out of range (max={data.shape[1]-1}), falling back to 0'
                )
                ch = 0
            return data[:, ch]
        else:
            self.get_logger().warning(
                f'Unknown mix_strategy "{strategy}", defaulting to mean'
            )
            return data.mean(axis=1)

    # ------------------------------------------------------------------ #
    # Timestamp metadata management
    # ------------------------------------------------------------------ #

    def _consume_ts_frames(self, n: int) -> TimestampAccumulator:
        """Advance timestamp tracking by *n* native frames and return
        a TimestampAccumulator positioned at the start of this segment.

        Must be called from the worker thread only.
        """
        # Snapshot accumulator position *before* advancing — that's the
        # timestamp for the start of the audio we just read.
        if self._native_ts_accum is not None:
            start_accum = self._native_ts_accum.copy()
        else:
            start_accum = None

        remaining = n
        while remaining > 0:
            if self._pending_ts_remaining == 0:
                with self._ts_lock:
                    if not self._ts_deque:
                        self.get_logger().error(
                            f'Timestamp metadata desync: {remaining} frames to consume '
                            f'but no timestamp segments left'
                        )
                        # Best effort: advance using current accumulator
                        if self._native_ts_accum is not None:
                            if start_accum is None:
                                start_accum = self._native_ts_accum.copy()
                            self._native_ts_accum.advance(remaining)
                        return start_accum
                    frames, start_ns = self._ts_deque.popleft()
                self._native_ts_accum = TimestampAccumulator(start_ns, self._native_sr)
                self._pending_ts_remaining = frames
                if start_accum is None:
                    start_accum = self._native_ts_accum.copy()

            consume = min(remaining, self._pending_ts_remaining)
            self._native_ts_accum.advance(consume)
            self._pending_ts_remaining -= consume
            remaining -= consume

        return start_accum

    def _discard_ts_frames(self, n: int):
        """Advance timestamp tracking past *n* dropped native frames."""
        remaining = n
        while remaining > 0:
            if self._pending_ts_remaining == 0:
                with self._ts_lock:
                    if not self._ts_deque:
                        self.get_logger().error(
                            f'Timestamp metadata desync: {remaining} frames to discard '
                            f'but no timestamp segments left'
                        )
                        if self._native_ts_accum is not None:
                            self._native_ts_accum.advance(remaining)
                        return
                    frames, start_ns = self._ts_deque.popleft()
                self._native_ts_accum = TimestampAccumulator(start_ns, self._native_sr)
                self._pending_ts_remaining = frames

            consume = min(remaining, self._pending_ts_remaining)
            self._native_ts_accum.advance(consume)
            self._pending_ts_remaining -= consume
            remaining -= consume

    # ------------------------------------------------------------------ #
    # Worker loop
    # ------------------------------------------------------------------ #

    def _worker_loop(self):
        """Worker thread: pop from ring buffer, mix, resample, publish."""
        output_cap_samples = int(_OUTPUT_CAP_S * self._target_sr)

        while self._running:
            # -- Stall detection --
            stall_ns = int(self._stall_timeout_s * 1e9)
            if self._last_callback_ns > 0:
                since_cb = time.monotonic_ns() - self._last_callback_ns
                if since_cb > stall_ns:
                    self.get_logger().fatal(
                        f'Audio callback stall detected '
                        f'({since_cb / 1e6:.0f}ms since last callback). '
                        f'Shutting down for respawn.'
                    )
                    self._running = False
                    return

            # -- Log and clear callback status counts --
            with self._status_lock:
                if self._status_counts:
                    for s, cnt in self._status_counts.items():
                        self.get_logger().warning(
                            f'PortAudio status: "{s}" x{cnt}'
                        )
                    self._status_counts.clear()

            # -- Check ring fill level --
            fill = self._ring.fill_level
            if fill > self._warn_threshold:
                now_m = time.monotonic()
                if now_m - self._last_warn_time_fill > 2.0:
                    self.get_logger().warning(
                        f'Ring buffer fill={fill:.0%} '
                        f'(dropped={self._ring.dropped_frames} total)'
                    )
                    self._last_warn_time_fill = now_m

            # -- Handle ring overflow --
            dropped_since_last = self._ring.dropped_frames - self._last_drop_count
            self._last_drop_count = self._ring.dropped_frames

            if dropped_since_last > 0:
                self._drops_ring_overflow += dropped_since_last
                now_m = time.monotonic()
                if now_m - self._last_warn_time_ring > 2.0:
                    self.get_logger().warning(
                        f'Ring overflow: {dropped_since_last} native frames dropped '
                        f'(total: {self._drops_ring_overflow})'
                    )
                    self._last_warn_time_ring = now_m

                # Consume dropped frames' timestamp metadata
                self._discard_ts_frames(dropped_since_last)

                # Advance output sample_index to reflect the gap
                dropped_output = dropped_since_last * self._target_sr // self._native_sr
                self._sample_index += dropped_output

            # -- Read from ring buffer --
            data = self._ring.read()
            if data.shape[0] == 0:
                time.sleep(_POLL_INTERVAL_S)
                continue

            native_frames_read = data.shape[0]

            # Consume timestamp metadata for frames we just read
            ts_accum_start = self._consume_ts_frames(native_frames_read)

            # -- Mono mix --
            mono = self._mono_mix(data)

            # -- Resample if needed --
            if self._resampler is not None:
                mono = self._resampler.resample_chunk(mono)
            elif self._native_sr != self._target_sr:
                continue

            if len(mono) == 0:
                continue

            # -- Build output-domain timestamp accumulator --
            if ts_accum_start is not None:
                out_ts = TimestampAccumulator(ts_accum_start.current, self._target_sr)
            elif self._output_ts_accum is not None:
                out_ts = self._output_ts_accum.copy()
            else:
                # Fallback: no timestamp info yet
                out_ts = TimestampAccumulator(time.monotonic_ns(), self._target_sr)

            # Append to output buffer with coupled metadata
            self._output_chunks.append((mono, self._sample_index, out_ts))
            self._output_samples += len(mono)
            self._sample_index += len(mono)

            # Update running output ts accumulator for next iteration
            out_ts_end = out_ts.copy()
            out_ts_end.advance(len(mono))
            self._output_ts_accum = out_ts_end

            # -- Enforce output buffer cap --
            while self._output_samples > output_cap_samples and self._output_chunks:
                oldest_audio, oldest_idx, oldest_ts = self._output_chunks[0]
                drop_n = len(oldest_audio)
                self._output_chunks.popleft()
                self._output_samples -= drop_n
                self._drops_output_cap += drop_n

                now_m = time.monotonic()
                if now_m - self._last_warn_time_output_cap > 2.0:
                    self.get_logger().warning(
                        f'Output buffer cap: dropped {drop_n} samples '
                        f'(total: {self._drops_output_cap})'
                    )
                    self._last_warn_time_output_cap = now_m

            # -- Publish complete chunks --
            self._publish_from_output_buf()

    def _publish_from_output_buf(self):
        """Slice and publish complete chunks from the output buffer."""
        # Flatten enough samples for at least one chunk
        while self._output_samples >= self._chunk_samples and self._output_chunks:
            # Collect samples for one chunk
            needed = self._chunk_samples
            parts = []
            chunk_start_idx = None
            chunk_ts_accum = None
            collected = 0

            while collected < needed and self._output_chunks:
                audio, start_idx, ts_accum = self._output_chunks[0]

                if chunk_start_idx is None:
                    chunk_start_idx = start_idx
                    chunk_ts_accum = ts_accum.copy()

                take = min(needed - collected, len(audio))
                parts.append(audio[:take])
                collected += take

                if take == len(audio):
                    self._output_chunks.popleft()
                else:
                    # Partial consume: update entry in-place
                    remainder = audio[take:]
                    new_idx = start_idx + take
                    new_ts = ts_accum.copy()
                    new_ts.advance(take)
                    self._output_chunks[0] = (remainder, new_idx, new_ts)

                self._output_samples -= take

            if collected < needed:
                break

            chunk = np.concatenate(parts) if len(parts) > 1 else parts[0]

            # Compute capture timestamps
            capture_mono_ns = chunk_ts_accum.current
            chunk_ts_accum.advance(len(chunk))
            capture_end_ns = chunk_ts_accum.current

            self._publish_chunk(chunk, chunk_start_idx, capture_mono_ns, capture_end_ns)

    def _publish_chunk(self, chunk: np.ndarray, sample_index: int,
                       capture_mono_ns: int, capture_end_ns: int):
        """Publish a mono audio chunk as an AudioData message."""
        # Apply gain and convert float32 [-1.0, 1.0] to int16 PCM
        pcm = np.clip(chunk * self._gain * 32767.0, -32768.0, 32767.0).astype(np.int16)

        msg = AudioData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sample_rate = self._target_sr
        msg.sample_index = sample_index
        msg.encoding = 'pcm_s16le'
        msg.capture_mono_ns = capture_mono_ns
        msg.capture_end_ns = capture_end_ns
        msg.data = pcm.tobytes()

        self._pub.publish(msg)

        # -- Jitter check --
        if self._prev_capture_end_ns is not None:
            jitter_ms = (capture_mono_ns - self._prev_capture_end_ns) / 1e6
            if abs(jitter_ms) > self._jitter_warn_ms:
                now_m = time.monotonic()
                if now_m - self._last_warn_time_jitter > 2.0:
                    self.get_logger().warning(
                        f'Callback timing jitter: {jitter_ms:.1f}ms'
                    )
                    self._last_warn_time_jitter = now_m

        # -- sample_index continuity check --
        if self._prev_sample_index is not None:
            expected = self._prev_sample_index + self._chunk_samples
            if sample_index != expected:
                self.get_logger().warning(
                    f'sample_index gap: expected {expected}, got {sample_index} '
                    f'(ring_drops={self._drops_ring_overflow}, '
                    f'cap_drops={self._drops_output_cap})'
                )

        self._prev_capture_end_ns = capture_end_ns
        self._prev_sample_index = sample_index

        # WAV dump
        if self._wav_file is not None:
            self._wav_file.writeframes(pcm.tobytes())

    # ------------------------------------------------------------------ #
    # Watchdog
    # ------------------------------------------------------------------ #

    def _watchdog_cb(self):
        """ROS timer callback: check if worker is still alive."""
        if not self._running:
            self.get_logger().warning('Watchdog: _running=False, shutting down node')
            rclpy.try_shutdown()

    # ------------------------------------------------------------------ #
    # Teardown
    # ------------------------------------------------------------------ #

    def destroy_node(self):
        self._running = False
        try:
            if self._stream is not None:
                self._stream.stop()
                self._stream.close()
        except Exception as e:
            self.get_logger().warning(f'Error closing stream: {e}')
        finally:
            self._stream = None

        if self._worker is not None:
            self._worker.join(timeout=3.0)
            if self._worker.is_alive():
                self.get_logger().fatal('Worker thread did not exit. Forcing exit.')
                import sys
                sys.stderr.flush()
                sys.stdout.flush()
                os._exit(1)

        if self._wav_file is not None:
            self._wav_file.close()
            self.get_logger().info('WAV dump closed')

        self.get_logger().info(
            f'Audio capture stopped. '
            f'Samples: {self._sample_index}. '
            f'Ring drops: {self._drops_ring_overflow}. '
            f'Output cap drops: {self._drops_output_cap}.'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
