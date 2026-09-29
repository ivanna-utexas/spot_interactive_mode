"""Speech-to-text node using Silero VAD + faster-whisper.

Subscribes to mono audio from audio_capture_node, detects speech segments
via Silero VAD (TorchScript), transcribes with faster-whisper, publishes
Transcription messages.

Features:
- Pre-roll / post-roll padding to capture clipped phonemes
- Echo/TTS gating via /spot_audio/tts_active (transient_local, with timeout)
- Bounded transcription queue (drop-oldest policy)
- Capture-time-based latency tracking on every transcription
- Stream epoch management: resets VAD + audio state on discontinuity/gating

Threading model: This node uses a SingleThreadedExecutor (rclpy default).
_on_audio and _on_tts_active never run concurrently.  The only true cross-thread
boundary is the transcription queue (queue.Queue, thread-safe).  All VAD state
lives in the executor thread.
"""

import collections
import queue
import threading
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool

from spot_audio_msgs.msg import AudioData, Transcription


class TimestampAccumulator:
    """Drift-free nanosecond timestamp tracking for arbitrary sample rates.

    See audio_capture_node.py for rationale.
    """

    __slots__ = ('ns', 'rem', 'sr')

    def __init__(self, start_ns: int, sample_rate: int):
        self.ns = start_ns
        self.rem = 0
        self.sr = sample_rate

    def advance(self, n_samples: int) -> int:
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


class SttNode(Node):

    def __init__(self):
        super().__init__('stt_node')

        # -- Parameters --
        self.declare_parameter('model_path', 'base')
        self.declare_parameter('model_cache_dir', '/opt/spot_audio_models/whisper-base')
        self.declare_parameter('language', 'en')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('compute_type', 'int8')

        self.declare_parameter('vad_model_path', '/opt/spot_audio_models/silero_vad.jit')
        self.declare_parameter('vad_threshold', 0.5)
        self.declare_parameter('vad_frame_size', 512)
        self.declare_parameter('min_speech_duration_ms', 250)
        self.declare_parameter('end_silence_ms', 400)
        self.declare_parameter('max_speech_duration_s', 30.0)
        self.declare_parameter('pre_speech_ms', 200)
        self.declare_parameter('post_speech_ms', 150)

        self.declare_parameter('tts_cooldown_ms', 500)
        self.declare_parameter('tts_timeout_s', 10.0)

        self.declare_parameter('max_pending_utterances', 2)
        self.declare_parameter('stale_threshold_ms', 5000.0)
        self.declare_parameter('log_transcripts_at_info', True)
        self.declare_parameter('jitter_warn_threshold_ms', 50.0)

        self._model_path = self.get_parameter('model_path').value
        self._model_cache_dir = self.get_parameter('model_cache_dir').value
        self._language = self.get_parameter('language').value
        self._whisper_device = self.get_parameter('device').value
        self._compute_type = self.get_parameter('compute_type').value

        self._vad_model_path = self.get_parameter('vad_model_path').value
        self._vad_threshold = self.get_parameter('vad_threshold').value
        self._vad_frame_size = self.get_parameter('vad_frame_size').value
        self._min_speech_ms = self.get_parameter('min_speech_duration_ms').value
        self._end_silence_ms = self.get_parameter('end_silence_ms').value
        self._max_speech_s = self.get_parameter('max_speech_duration_s').value
        self._pre_speech_ms = self.get_parameter('pre_speech_ms').value
        self._post_speech_ms = self.get_parameter('post_speech_ms').value

        self._tts_cooldown_ms = self.get_parameter('tts_cooldown_ms').value
        self._tts_timeout_s = self.get_parameter('tts_timeout_s').value

        self._max_pending = self.get_parameter('max_pending_utterances').value
        self._stale_threshold_ms = self.get_parameter('stale_threshold_ms').value
        self._log_transcripts_info = self.get_parameter('log_transcripts_at_info').value
        self._jitter_warn_ms = self.get_parameter('jitter_warn_threshold_ms').value

        # -- State --
        self._sample_rate = None  # set from first AudioData message

        # Audio accumulation buffer (deque of float32 arrays)
        self._audio_buf: collections.deque = collections.deque()
        self._audio_buf_samples: int = 0
        self._audio_buf_start_sample_index: int | None = None
        self._audio_buf_ts_accum: TimestampAccumulator | None = None

        # VAD state
        self._in_speech = False
        self._speech_start_time = None  # ROS time
        self._speech_start_mono = None  # monotonic time for latency
        self._speech_frames = []
        self._speech_sample_count = 0
        self._silence_sample_count = 0

        # Capture-time of last voiced frame (from TimestampAccumulator)
        self._capture_last_voiced_ns: int | None = None

        # Pre-roll: circular buffer of recent audio
        self._pre_roll_buf: collections.deque = collections.deque()
        self._pre_roll_max_samples: int = 0

        # Post-roll state
        self._collecting_post_roll = False
        self._post_roll_samples_needed: int = 0
        self._post_roll_collected: int = 0

        # Echo gating
        self._tts_active = False
        self._tts_cooldown_until: float = 0.0
        self._tts_last_msg_time: float | None = None

        # Stream epoch: bumped on discontinuity / gating activation
        self._stream_epoch: int = 0
        self._stale_epoch: bool = False
        self._epoch_msg_count: int = 0

        # Discontinuity detection
        self._last_sample_index: int | None = None
        self._last_chunk_samples: int = 0

        # Transcription queue (bounded, drop-oldest)
        self._tx_queue: queue.Queue = queue.Queue(maxsize=self._max_pending)

        # Drop counters
        self._drops_queue_full: int = 0
        self._drops_epoch_mismatch: int = 0
        self._drops_gating: int = 0

        # Whisper EMA
        self._avg_whisper_ms: float = 0.0
        self._whisper_ema_alpha: float = 0.3

        # Warning throttle
        self._last_warn_time_jitter: float = 0.0

        # -- Load models --
        self._vad_model = None
        self._vad_has_reset = False
        self._whisper_model = None
        self._load_vad()
        self._load_whisper()

        # -- Publishers / Subscribers --
        audio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(AudioData, '/spot_audio/audio', self._on_audio, audio_qos)

        tts_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(Bool, '/spot_audio/tts_active', self._on_tts_active, tts_qos)

        tx_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self._pub = self.create_publisher(Transcription, '/spot_audio/transcription', tx_qos)

        # -- Worker thread --
        self._running = True
        self._worker = threading.Thread(target=self._transcription_worker, daemon=True)
        self._worker.start()

        self.get_logger().info('stt_node started')

    # ------------------------------------------------------------------ #
    # Model loading
    # ------------------------------------------------------------------ #

    def _load_vad(self):
        """Load Silero VAD TorchScript model."""
        import torch
        self.get_logger().info(f'Loading Silero VAD from {self._vad_model_path}')
        try:
            self._vad_model = torch.jit.load(self._vad_model_path)
            self._vad_model.eval()
            self._vad_has_reset = hasattr(self._vad_model, 'reset_states')
            if not self._vad_has_reset:
                self.get_logger().warning(
                    'VAD model does not support reset_states(); state bleed risk'
                )
            self.get_logger().info('Silero VAD loaded (TorchScript)')
        except Exception as e:
            self.get_logger().fatal(f'Failed to load Silero VAD: {e}')
            raise

    def _load_whisper(self):
        """Load faster-whisper model."""
        self.get_logger().info(
            f'Loading Whisper model from {self._model_path} '
            f'(device={self._whisper_device}, compute_type={self._compute_type})'
        )
        try:
            from faster_whisper import WhisperModel
            self._whisper_model = WhisperModel(
                self._model_path,
                device=self._whisper_device,
                compute_type=self._compute_type,
                download_root=self._model_cache_dir,
            )
            self.get_logger().info('Whisper model loaded')
        except Exception as e:
            self.get_logger().fatal(f'Failed to load Whisper model: {e}')
            raise

    # ------------------------------------------------------------------ #
    # First-audio initialization
    # ------------------------------------------------------------------ #

    def _init_from_first_audio(self, msg: AudioData):
        """Initialize sample-rate-dependent state from first audio message."""
        self._sample_rate = msg.sample_rate

        if self._sample_rate != 16000:
            self.get_logger().warning(
                f'Audio sample rate is {self._sample_rate} Hz but Silero VAD '
                f'expects 16000 Hz. VAD behavior may be incorrect.'
            )

        frame_ms = self._vad_frame_size / self._sample_rate * 1000
        self.get_logger().info(
            f'Audio stream detected: sr={self._sample_rate}, '
            f'vad_frame_size={self._vad_frame_size} '
            f'({frame_ms:.1f}ms at {self._sample_rate} Hz)'
        )

        self._pre_roll_max_samples = int(self._sample_rate * self._pre_speech_ms / 1000)
        self._post_roll_samples_needed = int(self._sample_rate * self._post_speech_ms / 1000)

    # ------------------------------------------------------------------ #
    # Epoch management
    # ------------------------------------------------------------------ #

    def _bump_epoch(self, reason: str):
        """Centralized epoch bump — single entry point for discontinuity/gating resets."""
        self._stream_epoch += 1
        self.get_logger().warning(
            f'Stream epoch {self._stream_epoch} (reason: {reason})'
        )
        self._reset_vad_state()
        self._stale_epoch = False
        self._epoch_msg_count = 0

    def _reset_vad_state(self):
        """Reset VAD tracking state."""
        self._in_speech = False
        self._speech_sample_count = 0
        self._silence_sample_count = 0
        self._speech_frames = []
        self._collecting_post_roll = False
        self._post_roll_collected = 0
        self._capture_last_voiced_ns = None

        # Clear stale audio accumulation buffer + origin tracking
        self._audio_buf.clear()
        self._audio_buf_samples = 0
        self._audio_buf_start_sample_index = None
        self._audio_buf_ts_accum = None

        # Reset Silero VAD LSTM hidden states
        if self._vad_has_reset:
            self._vad_model.reset_states()

    # ------------------------------------------------------------------ #
    # Echo gating
    # ------------------------------------------------------------------ #

    def _is_gated(self) -> bool:
        """Check if audio should be dropped due to TTS gating."""
        now = time.monotonic()

        # Check TTS timeout (dead publisher protection)
        if self._tts_last_msg_time is not None:
            if now - self._tts_last_msg_time > self._tts_timeout_s:
                if self._tts_active:
                    self.get_logger().warning(
                        f'TTS timeout ({self._tts_timeout_s}s without msg) — disabling gate'
                    )
                    self._tts_active = False

        if self._tts_active:
            return True

        if now < self._tts_cooldown_until:
            return True

        return False

    def _on_tts_active(self, msg: Bool):
        """Handle TTS gating state updates."""
        now = time.monotonic()
        self._tts_last_msg_time = now

        was_active = self._tts_active
        self._tts_active = msg.data

        if was_active and not msg.data:
            # TTS just ended — start cooldown
            self._tts_cooldown_until = now + self._tts_cooldown_ms / 1000.0
            self.get_logger().debug(
                f'TTS ended, cooldown {self._tts_cooldown_ms}ms'
            )
            self._reset_vad_state()

        if not was_active and msg.data:
            self.get_logger().debug('TTS active — gating audio')
            self._bump_epoch('TTS gating activation')

    # ------------------------------------------------------------------ #
    # Audio callback
    # ------------------------------------------------------------------ #

    def _on_audio(self, msg: AudioData):
        """Process incoming audio chunk."""
        if self._sample_rate is None:
            self._init_from_first_audio(msg)

        # -- Sample rate change detection --
        if msg.sample_rate != self._sample_rate:
            self.get_logger().warning(
                f'Sample rate changed: {self._sample_rate} -> {msg.sample_rate}'
            )
            self._sample_rate = msg.sample_rate
            self._pre_roll_max_samples = int(self._sample_rate * self._pre_speech_ms / 1000)
            self._post_roll_samples_needed = int(self._sample_rate * self._post_speech_ms / 1000)
            self._bump_epoch('sample rate change')

        # -- Stale capture timestamp detection (epoch-start only) --
        if msg.capture_end_ns > 0:
            recv_ns = time.monotonic_ns()
            age_at_receive_ms = (recv_ns - msg.capture_end_ns) / 1e6

            # Negative age = monotonic domains don't match
            if age_at_receive_ms < -1000:
                if not self._stale_epoch:
                    self.get_logger().warning(
                        f'Negative capture age ({age_at_receive_ms:.0f}ms). '
                        f'Monotonic domains mismatch. E2E suppressed.'
                    )
                    self._stale_epoch = True

            # Only evaluate staleness during first 10 messages of an epoch
            if self._epoch_msg_count < 10:
                if age_at_receive_ms > self._stale_threshold_ms:
                    if not self._stale_epoch:
                        self.get_logger().warning(
                            f'Stale capture timestamps at epoch start '
                            f'(age={age_at_receive_ms:.0f}ms). '
                            f'E2E suppressed (likely bag replay).'
                        )
                        self._stale_epoch = True

        self._epoch_msg_count += 1

        # -- Discontinuity detection (forward + backward) --
        chunk_samples = len(msg.data) // 2  # int16 = 2 bytes per sample
        if self._last_sample_index is not None and self._last_chunk_samples > 0:
            expected_next = self._last_sample_index + self._last_chunk_samples
            gap_threshold = 2 * self._last_chunk_samples

            if msg.sample_index < self._last_sample_index:
                self._bump_epoch('sample_index backwards jump')
            elif msg.sample_index >= expected_next + gap_threshold:
                self._bump_epoch(
                    f'sample_index forward gap '
                    f'(expected ~{expected_next}, got {msg.sample_index})'
                )

        self._last_sample_index = msg.sample_index
        self._last_chunk_samples = chunk_samples

        # Decode PCM int16 to float32
        pcm = np.frombuffer(bytes(msg.data), dtype=np.int16).astype(np.float32) / 32768.0

        # Echo gating: drop audio during TTS
        if self._is_gated():
            self._drops_gating += 1
            return

        # Collect post-roll if in that phase
        if self._collecting_post_roll:
            self._speech_frames.append(pcm)
            self._post_roll_collected += len(pcm)
            if self._post_roll_collected >= self._post_roll_samples_needed:
                self._finalize_utterance()
            return

        # Update pre-roll buffer (always)
        self._pre_roll_buf.append(pcm)
        total_pre = sum(len(c) for c in self._pre_roll_buf)
        while total_pre > self._pre_roll_max_samples and len(self._pre_roll_buf) > 1:
            removed = self._pre_roll_buf.popleft()
            total_pre -= len(removed)

        # Run VAD frame by frame
        self._process_vad(pcm, msg.header.stamp, msg)

    # ------------------------------------------------------------------ #
    # VAD processing
    # ------------------------------------------------------------------ #

    def _process_vad(self, audio: np.ndarray, stamp, msg: AudioData):
        """Run Silero VAD on audio, detect speech boundaries."""
        import torch

        # Initialize buffer-origin tracking when buffer transitions from empty
        if self._audio_buf_samples == 0:
            self._audio_buf_start_sample_index = msg.sample_index
            if msg.capture_mono_ns > 0:
                self._audio_buf_ts_accum = TimestampAccumulator(
                    msg.capture_mono_ns, self._sample_rate
                )
            else:
                self._audio_buf_ts_accum = None

        self._audio_buf.append(audio)
        self._audio_buf_samples += len(audio)

        while self._audio_buf_samples >= self._vad_frame_size:
            frame, frame_end_ns = self._extract_frame()

            # Run VAD
            frame_tensor = torch.from_numpy(frame).unsqueeze(0)
            with torch.no_grad():
                prob = self._vad_model(frame_tensor, self._sample_rate).item()

            is_speech = prob >= self._vad_threshold

            if not self._in_speech:
                if is_speech:
                    self._speech_sample_count += self._vad_frame_size
                    if self._speech_sample_count >= int(
                        self._sample_rate * self._min_speech_ms / 1000
                    ):
                        self._in_speech = True
                        self._speech_start_time = stamp
                        self._speech_start_mono = time.monotonic()
                        self._silence_sample_count = 0

                        # Include pre-roll
                        self._speech_frames = list(self._pre_roll_buf)

                        self.get_logger().debug('Speech start detected')
                else:
                    self._speech_sample_count = 0
            else:
                # In speech
                self._speech_frames.append(frame)

                if is_speech and frame_end_ns is not None:
                    self._capture_last_voiced_ns = frame_end_ns

                if not is_speech:
                    self._silence_sample_count += self._vad_frame_size
                    end_silence_samples = int(
                        self._sample_rate * self._end_silence_ms / 1000
                    )
                    if self._silence_sample_count >= end_silence_samples:
                        self._start_post_roll()
                else:
                    self._silence_sample_count = 0

                # Max duration check
                total_speech_samples = sum(len(f) for f in self._speech_frames)
                if total_speech_samples >= int(self._sample_rate * self._max_speech_s):
                    self.get_logger().info('Max speech duration reached, force transcribing')
                    self._start_post_roll()

    def _extract_frame(self) -> tuple:
        """Extract exactly one VAD frame from the audio accumulation buffer.

        Returns:
            (frame_audio, frame_end_capture_ns): The audio frame and the
            estimated capture time of the frame's last sample (or None if
            no timestamp tracking is available).
        """
        needed = self._vad_frame_size
        chunks = []
        collected = 0

        while collected < needed and self._audio_buf:
            chunk = self._audio_buf[0]
            remaining = needed - collected
            if len(chunk) <= remaining:
                chunks.append(chunk)
                collected += len(chunk)
                self._audio_buf.popleft()
                self._audio_buf_samples -= len(chunk)
            else:
                chunks.append(chunk[:remaining])
                self._audio_buf[0] = chunk[remaining:]
                self._audio_buf_samples -= remaining
                collected += remaining

        frame = np.concatenate(chunks) if len(chunks) > 1 else chunks[0]

        # Advance buffer-origin tracking
        frame_end_ns = None
        if self._audio_buf_ts_accum is not None:
            frame_end_ns = self._audio_buf_ts_accum.advance(needed)
        if self._audio_buf_start_sample_index is not None:
            self._audio_buf_start_sample_index += needed

        return frame, frame_end_ns

    # ------------------------------------------------------------------ #
    # Utterance finalization
    # ------------------------------------------------------------------ #

    def _start_post_roll(self):
        """Begin collecting post-roll audio after speech end."""
        self._collecting_post_roll = True
        self._post_roll_collected = 0
        self._in_speech = False
        self._speech_sample_count = 0
        self._silence_sample_count = 0
        self.get_logger().debug('Speech end, collecting post-roll')

        if self._post_roll_samples_needed == 0:
            self._finalize_utterance()

    def _finalize_utterance(self):
        """Finalize utterance and push to transcription queue."""
        self._collecting_post_roll = False

        if not self._speech_frames:
            self._reset_vad_state()
            return

        audio = np.concatenate(self._speech_frames)
        duration = len(audio) / self._sample_rate

        if duration < self._min_speech_ms / 1000.0:
            self.get_logger().debug(f'Utterance too short ({duration:.2f}s), discarding')
            self._reset_vad_state()
            return

        queued_ns = time.monotonic_ns()

        utterance = {
            'audio': audio,
            'duration': duration,
            'start_time': self._speech_start_time,
            'end_time': self.get_clock().now().to_msg(),
            'speech_start_mono': self._speech_start_mono,
            'capture_last_voiced_ns': self._capture_last_voiced_ns,
            'queued_ns': queued_ns,
            'epoch': self._stream_epoch,
        }

        # Push to bounded queue (drop oldest if full)
        try:
            self._tx_queue.put_nowait(utterance)
        except queue.Full:
            try:
                self._tx_queue.get_nowait()
                self._drops_queue_full += 1
                self.get_logger().warning(
                    f'Transcription queue full, dropped oldest '
                    f'(total dropped: {self._drops_queue_full})'
                )
                self._tx_queue.put_nowait(utterance)
            except queue.Empty:
                pass

        self._reset_vad_state()

    # ------------------------------------------------------------------ #
    # Transcription worker
    # ------------------------------------------------------------------ #

    def _transcription_worker(self):
        """Worker thread: dequeue utterances, run Whisper, publish."""
        while self._running:
            try:
                utterance = self._tx_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            dequeued_ns = time.monotonic_ns()

            # Pre-inference epoch check
            if utterance['epoch'] != self._stream_epoch:
                self._drops_epoch_mismatch += 1
                self.get_logger().info(
                    f'Dropping stale utterance (epoch {utterance["epoch"]}, '
                    f'current {self._stream_epoch})'
                )
                continue

            t_start = time.monotonic_ns()

            try:
                segments, info = self._whisper_model.transcribe(
                    utterance['audio'],
                    language=self._language if self._language else None,
                    beam_size=5,
                    vad_filter=False,
                )

                text_parts = []
                total_logprob = 0.0
                total_segments = 0
                no_speech_prob = 0.0

                for seg in segments:
                    text_parts.append(seg.text.strip())
                    total_logprob += seg.avg_logprob
                    no_speech_prob = max(no_speech_prob, seg.no_speech_prob)
                    total_segments += 1

                text = ' '.join(text_parts).strip()
                avg_logprob = total_logprob / max(total_segments, 1)

            except Exception as e:
                self.get_logger().error(f'Whisper transcription failed: {e}')
                continue

            stt_done_ns = time.monotonic_ns()
            whisper_ms = (stt_done_ns - t_start) / 1e6

            # Update EMA
            if self._avg_whisper_ms == 0.0:
                self._avg_whisper_ms = whisper_ms
            else:
                self._avg_whisper_ms = (
                    self._whisper_ema_alpha * whisper_ms
                    + (1 - self._whisper_ema_alpha) * self._avg_whisper_ms
                )

            # Post-inference epoch check
            if utterance['epoch'] != self._stream_epoch:
                self._drops_epoch_mismatch += 1
                self.get_logger().info(
                    f'Dropping utterance transcribed during epoch transition'
                )
                continue

            if not text:
                self.get_logger().debug('Empty transcription, skipping')
                continue

            # Publish
            published_ns = time.monotonic_ns()

            msg = Transcription()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.utterance_id = str(uuid.uuid4())
            msg.text = text
            msg.language = self._language or info.language or ''
            msg.start_time = utterance['start_time']
            msg.end_time = utterance['end_time']
            msg.audio_duration = utterance['duration']
            msg.avg_logprob = avg_logprob
            msg.no_speech_prob = no_speech_prob
            msg.is_final = True

            self._pub.publish(msg)

            # -- Latency metrics --
            queued_ns = utterance['queued_ns']
            queue_wait_ms = (dequeued_ns - queued_ns) / 1e6
            pipeline_ms = (published_ns - queued_ns) / 1e6
            qsize = self._tx_queue.qsize()
            est_backlog_ms = self._avg_whisper_ms * qsize

            capture_last_voiced_ns = utterance.get('capture_last_voiced_ns')
            if capture_last_voiced_ns is not None and not self._stale_epoch:
                e2e_ms = (published_ns - capture_last_voiced_ns) / 1e6
                e2e_str = f'{e2e_ms:.0f}ms'
            elif self._stale_epoch:
                e2e_str = 'N/A (stale)'
            else:
                e2e_str = 'N/A'

            metrics = (
                f'e2e={e2e_str} | '
                f'pipeline={pipeline_ms:.0f}ms | '
                f'queue_wait={queue_wait_ms:.0f}ms | '
                f'whisper={whisper_ms:.0f}ms | '
                f'epoch={self._stream_epoch} | '
                f'qsize={qsize} | '
                f'est_backlog={est_backlog_ms:.0f}ms | '
                f'duration={utterance["duration"]:.1f}s | '
                f'logprob={avg_logprob:.2f} | '
                f'no_speech={no_speech_prob:.2f}'
            )

            if self._log_transcripts_info:
                self.get_logger().info(f'Transcription: "{text}" | {metrics}')
            else:
                self.get_logger().info(f'Transcription metrics: {metrics}')
                self.get_logger().debug(f'Transcript text: "{text}"')

    # ------------------------------------------------------------------ #
    # Teardown
    # ------------------------------------------------------------------ #

    def destroy_node(self):
        self._running = False
        if self._worker is not None:
            self._worker.join(timeout=5.0)
        self.get_logger().info(
            f'STT node stopped. '
            f'Epoch: {self._stream_epoch}. '
            f'Queue drops: {self._drops_queue_full}. '
            f'Epoch mismatch drops: {self._drops_epoch_mismatch}. '
            f'Gating drops: {self._drops_gating}.'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SttNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
