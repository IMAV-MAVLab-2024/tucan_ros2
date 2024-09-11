uint8 id # id of the marker (set to zero if no marker is detected)
bool detected # if detected is true, a marker is detected in the image
int32 x # x position of the marker in the image
int32 y # y position of the marker in the image
int32 previou_x # x position of the marker before lost image
int32 previou_y # y position of the marker before lost image
float32 x_global # x position of the marker in the real world
float32 y_global # y position of the marker in the real world
float32 z_global # z position of the marker in the real world
float32 previou_x_global # x position of the marker before lost image in the real world
float32 previou_y_global # y position of the marker before lost image in the real world
float32 previou_z_global # z position of the marker before lost image in the real world