#!/usr/bin/env python3
import time
from pymavlink import mavutil
from pynput import keyboard


# 1) CONNECT & HEARTBEAT

the_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
the_connection.wait_heartbeat()
print(f" Heartbeat from sys:{the_connection.target_system} comp:{the_connection.target_component}")

# 2) GUIDED MODE --> ARM --> TAKEOFF

the_connection.set_mode_apm('GUIDED')
print("Mode -> GUIDED")
the_connection.arducopter_arm()
print("Arming motors…")
time.sleep(2)

takeoff_alt = 1.0
the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,        # confirm
    0,0,0,0,  # unused
    0,0,      # lat, lon ignored
    takeoff_alt
)
print(f"Taking off to {takeoff_alt} m…")
time.sleep(5)

# 3) VELOCITY + YAW_RATE HOLD (mask=1479) 

MASK_VEL_YAWRATE = 1479
FRAME_BODY_OFFSET_NED = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED  # 9

def set_velocity_body(vx, vy, vz):
    """
    Send body-frame (forward, right, down) velocity AND enforce yaw_rate=0.
    """

    the_connection.mav.set_position_target_local_ned_send(
        0,
        the_connection.target_system,
        the_connection.target_component,
        FRAME_BODY_OFFSET_NED,
        MASK_VEL_YAWRATE,
        0,0,0,        # ignore position
        vx, vy, vz,   # desired m/s
        0,0,0,        # ignore acceleration
        0,             # yaw ignored
        0              # yaw_rate = 0 --> hold heading
    )

def condition_yaw(target_heading, direction, rate, relative=True):
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        target_heading,
        rate,
        direction,
        1 if relative else 0,
        0,0,0
    )

# 4) KEYBOARD LISTENER

gnd_speed     = 0.5
height_delta  = 0.5
max_speed     = 3.0
yaw_rate      = 20
heading       = 30   # relative yaw target
key_pressed   = '#'

def on_press(key):
    global key_pressed
    try:
        key_pressed = key.char.lower()
    except AttributeError:
        pass

def on_release(key):
    global gnd_speed, key_pressed
    set_velocity_body(0, 0, 0)
    gnd_speed = 0.0
    key_pressed = '#'

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

print("→ Ready: W/A/S/D = strafe, Z/X = down/up, Q/E = yaw. Ctrl-C to exit.")

# 5) MAIN LOOP
try:
    while True:
        if key_pressed != '#':
            if gnd_speed < max_speed:
                gnd_speed += 0.05

            cmd = key_pressed
            if   cmd == 'w': set_velocity_body( gnd_speed, 0, 0)
            elif cmd == 's': set_velocity_body(-gnd_speed, 0, 0)
            elif cmd == 'a': set_velocity_body(0, -gnd_speed, 0)
            elif cmd == 'd': set_velocity_body(0,  gnd_speed, 0)
            elif cmd == 'z': set_velocity_body(0, 0, -height_delta)
            elif cmd == 'x': set_velocity_body(0, 0,  height_delta)
            elif cmd == 'q': condition_yaw(heading, -1, yaw_rate)
            elif cmd == 'e': condition_yaw(heading, +1, yaw_rate)

            key_pressed = '#'

        time.sleep(0.02)

except KeyboardInterrupt:
    print("\n Exiting…")
    listener.stop()