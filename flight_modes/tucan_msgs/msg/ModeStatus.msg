# Modes output their current state

tucan_msgs/Mode mode

uint8 MODE_ERROR = 0         # Something went wrong
uint8 MODE_INACTIVE = 1      # Mode is inactive
uint8 MODE_ACTIVE = 2        # Mode is active
uint8 MODE_FINISHED = 3      # Mode has finished

uint8 mode_status 1

bool busy false              # wether or not the mode can be interrupted