"""Tool: Record audio from /spot_audio/audio topic to a WAV file.

Subscribes to the AudioData topic, writes PCM data to a WAV file.
Used for validating that capture + resample pipeline produces intelligible audio.

Usage:
    ros2 run spot_audio record_wav                          # default: 10s to /tmp/spot_audio_recording.wav
    ros2 run spot_audio record_wav --ros-args -p duration:=30 -p output_path:=/tmp/test.wav
"""

import wave

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from spot_audio_msgs.msg import AudioData


class RecordWavNode(Node):

    def __init__(self):
        super().__init__('record_wav')

        self.declare_parameter('duration', 10.0)
        self.declare_parameter('output_path', '/tmp/spot_audio_recording.wav')

        self._duration = self.get_parameter('duration').value
        self._output_path = self.get_parameter('output_path').value

        self._wav_file = None
        self._samples_written = 0
        self._sample_rate = None
        self._target_samples = None
        self._prev_sample_index = None
        self._gap_count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(AudioData, '/spot_audio/audio', self._on_audio, qos)

        self.get_logger().info(
            f'Recording {self._duration}s to {self._output_path} ...'
        )

    def _on_audio(self, msg: AudioData):
        if self._wav_file is None:
            # Initialize WAV on first message
            self._sample_rate = msg.sample_rate
            self._target_samples = int(self._sample_rate * self._duration)
            self._wav_file = wave.open(self._output_path, 'wb')
            self._wav_file.setnchannels(1)
            self._wav_file.setsampwidth(2)  # 16-bit
            self._wav_file.setframerate(self._sample_rate)
            self.get_logger().info(
                f'WAV initialized: sr={self._sample_rate}, encoding={msg.encoding}'
            )

        # Check sample continuity
        if self._prev_sample_index is not None:
            expected = self._prev_sample_index
            if msg.sample_index != expected:
                gap = msg.sample_index - expected
                self._gap_count += 1
                self.get_logger().warn(
                    f'Sample discontinuity: expected {expected}, got {msg.sample_index} '
                    f'(gap={gap} samples, {gap/self._sample_rate:.3f}s)'
                )

        pcm_bytes = bytes(msg.data)
        num_samples = len(pcm_bytes) // 2  # 16-bit = 2 bytes per sample
        self._prev_sample_index = msg.sample_index + num_samples

        self._wav_file.writeframes(pcm_bytes)
        self._samples_written += num_samples

        # Progress
        elapsed = self._samples_written / self._sample_rate
        if int(elapsed) % 2 == 0 and int(elapsed) > 0:
            # Log every 2 seconds
            remaining = self._duration - elapsed
            if remaining > 0 and abs(elapsed - round(elapsed)) < 0.1:
                self.get_logger().info(f'Recording... {elapsed:.0f}s / {self._duration:.0f}s')

        if self._samples_written >= self._target_samples:
            self._wav_file.close()
            self.get_logger().info(
                f'Recording complete: {self._output_path} '
                f'({self._samples_written} samples, '
                f'{self._samples_written/self._sample_rate:.1f}s, '
                f'gaps={self._gap_count})'
            )
            raise SystemExit(0)

    def destroy_node(self):
        if self._wav_file is not None:
            self._wav_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecordWavNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
