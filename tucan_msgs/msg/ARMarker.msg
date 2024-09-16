
bool detected # if detected is true, a marker is detected in the image

uint16 id # id of the marker (set to zero only if there never has been a marker detection before, set to current marker id if currently detecting, set to last seen marker id if previously detected)

int32 x # x position of the marker in the image
int32 y # y position of the marker in the image

float32 yaw # radians of the ned yaw of the ar marker

float32 x_global # x position of the marker in the real world
float32 y_global # y position of the marker in the real world
float32 z_global # z position of the marker in the real world

builtin_interfaces/Time last_detection_timestamp   # when the last detection of an aruco marker happened, 