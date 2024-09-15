bool detected

float32 x_picture       # x position of the center of the line in the image
float32 y_picture       # x position of the center of the line in the image

float32 lateral_offset

float32 x_offset_dir        # x position of the center of the line in NED
float32 y_offset_dir        # y position of the center of the line in NED
float32 yaw             # yaw of the line in NED

builtin_interfaces/Time last_detection_timestamp   # when the last detection of an aruco object happened, 

# float32 x_global_end # x position of the object in the image
# float32 y_global_end # end point

# float32 x_global_start # x position of the object in the real world
# float32 y_global_start # 

