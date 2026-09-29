# Speech transcription result from the STT pipeline.
#
# avg_logprob and no_speech_prob are raw Whisper metrics — no synthetic confidence.
# is_final is true for completed utterances (false reserved for future streaming partials).

std_msgs/Header header
string utterance_id                       # UUID for this utterance
string text                               # transcribed speech
string language                           # language code (e.g. "en")
builtin_interfaces/Time start_time        # utterance start (from audio timestamps)
builtin_interfaces/Time end_time          # utterance end
float64 audio_duration                    # duration of audio segment in seconds
float32 avg_logprob                       # Whisper average log probability (raw)
float32 no_speech_prob                    # Whisper no-speech probability (raw)
bool is_final                             # true for final transcript
