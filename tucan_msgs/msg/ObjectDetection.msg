
bool detected # if detected is true, a marker is detected in the image

int32 x # x position of the object in the image
int32 y # y position of the object in the image

float32 x_global # x position of the object in the real world
float32 y_global # y position of the object in the real world
float32 z_global # z position of the object in the real world
float32 yaw_global # yaw position of the object in the real world

builtin_interfaces/Time last_detection_timestamp   # when the last detection of an aruco object happened, 