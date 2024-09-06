Header header
int8 servo_id    # sequence ID: serial id of the servo from which the feedback is from
int16 position   # position of the servo
int16 speed
int16 load        # motor load(0~1000, 1000 = 100% max load)
int8 supply_volt
int8 temp        # degrees in celsius