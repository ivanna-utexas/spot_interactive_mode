# Mono audio chunk from the microphone capture pipeline.
#
# header.stamp = ROS publish time (wall clock).
# sample_index is in the output domain (at sample_rate, post-resample).
# Monotonically increasing within a capture session; jumps forward on
# ring overflow or output buffer cap drops; resets to 0 on device reconnect.
#
# capture_mono_ns / capture_end_ns: estimated monotonic clock times of the
# first and one-past-last sample in this chunk.  Computed by back-dating
# the sounddevice callback time by block duration.  Usable only for deltas
# on the same machine (not ROS time).  Set to 0 if unavailable.

std_msgs/Header header
uint32 sample_rate             # output sample rate (e.g. 16000)
uint64 sample_index            # index of first output sample in continuous stream
string encoding                # "pcm_s16le" (PCM 16-bit signed little-endian)
uint64 capture_mono_ns         # estimated monotonic time of first sample (ns)
uint64 capture_end_ns          # estimated monotonic time one-past-last sample (ns)
uint8[] data                   # raw PCM bytes, always mono
