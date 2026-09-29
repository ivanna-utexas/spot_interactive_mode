std_msgs/Header header
float32 x          # normalized horizontal gaze [-1, 1]
float32 y          # normalized vertical gaze [-1, 1]
bool detected      # true if a face is currently tracked
uint32 num_faces   # total faces seen this frame
