# =========================
# SPIKE PRIME FLL – Smooth Gyro Drive + Smooth Turns
# Ports: Left motor = E, Right motor = A(όπως το έχεις)
# Χωρίς anchors (μόνο gyro + encoders)
# =========================

from hub import port, motion_sensor
import runloop, motor, motor_pair

# ----- MOTORS -----
PAIR = motor_pair.PAIR_1
LEFT = port.E
RIGHT = port.A
motor_pair.pair(PAIR, LEFT, RIGHT)

# ----- TUNING (ρύθμισέ τα αν χρειαστεί) -----
WHEEL_CIRCUMFERENCE_CM = 17.6# κράτα το δικό σου
KP_STRAIGHT = 1.2            # P για ευθεία
KD_STRAIGHT = 0.18            # D 
MAX_STEER = 100                # 

TURN_FAST = 180                # default fast turn speed
TURN_SLOW = 90                # slowdown κοντά στο στόχο
TURN_SLOW_ZONE_DEG = 8        # πόσες μοίρες πριν τον στόχο να κόψει

DRIVE_SLOW_ZONE_DEG = 160    # πόσες "μοίρες τροχού" πριν το τέλος να κόψει ταχύτητα
DRIVE_MIN_VEL = 140            # ελάχιστη ταχύτητα στο τέλος (για να μην προσπερνάει)


# ----- HELPERS -----
def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x

def yaw_deg():
    # όπως το έχεις: tilt_angles()[0] * -0.1
    return motion_sensor.tilt_angles()[0] * -0.1

async def gyro_reset_and_wait():
    motion_sensor.reset_yaw(0)
    await runloop.until(motion_sensor.stable)

def cm_to_motor_deg(cm):
    return int((abs(cm) / WHEEL_CIRCUMFERENCE_CM) * 360)


# =========================
# SMOOTH STRAIGHT (gyro + encoders + slowdown)
# =========================
async def gyro_straight_cm(cm, velocity=300, min_velocity=DRIVE_MIN_VEL):
    direction = 1 if cm >= 0 else -1
    target_deg = cm_to_motor_deg(cm)

    await gyro_reset_and_wait()

    # μετράμε από τον RIGHT encoder (όπως έκανες)
    motor.reset_relative_position(RIGHT, 0)

    prev_error = 0

    while True:
        pos = abs(motor.relative_position(RIGHT))
        remaining = target_deg - pos
        if remaining <= 0:
            break

        error = yaw_deg()                # θέλουμε 0
        derr = error - prev_error
        prev_error = error

        steer = int(-(KP_STRAIGHT * error + KD_STRAIGHT * derr))
        steer = clamp(steer, -MAX_STEER, MAX_STEER)

        # slowdown κοντά στο τέλος
        v = velocity
        if remaining < DRIVE_SLOW_ZONE_DEG:
            v = min_velocity

        motor_pair.move(PAIR, steer, velocity=direction * v)
        await runloop.sleep_ms(10)

    motor_pair.stop(PAIR)


# =========================
# SMOOTH TURNS (gyro + slowdown near target)
# =========================
async def gyro_turn_right(deg, fast=TURN_FAST, slow=TURN_SLOW, slow_zone=TURN_SLOW_ZONE_DEG):
    target = abs(deg)
    await gyro_reset_and_wait()

    while True:
        y = yaw_deg()
        remaining = target - y
        if remaining <= 0:
            break

        v = slow if remaining < slow_zone else fast
        motor_pair.move_tank(PAIR, v, -v)
        await runloop.sleep_ms(5)

    motor_pair.stop(PAIR)

async def gyro_turn_left(deg, fast=TURN_FAST, slow=TURN_SLOW, slow_zone=TURN_SLOW_ZONE_DEG):
    target = -abs(deg)
    await gyro_reset_and_wait()

    while True:
        y = yaw_deg()
        remaining = y - target# πόσο απέχει από το αρνητικό target
        if remaining <= 0:
            break

        v = slow if remaining < slow_zone else fast
        motor_pair.move_tank(PAIR, -v, v)
        await runloop.sleep_ms(5)

    motor_pair.stop(PAIR)


# =========================
#pause
# =========================
async def settle(ms=80):
    await runloop.sleep_ms(ms)


# =========================
#MAIN ΒΑΖΩ ΤΑ MISSION MOY 
# =========================
async def main():
    
    await gyro_straight_cm(30, velocity=320)
    await settle()

    

    await gyro_turn_left(20, fast=180, slow=90)
    await settle()

    await gyro_straight_cm(-30, velocity=320)
    await settle()

    await gyro_turn_right(20, fast=180, slow=90)
    await settle()

   #await armup()
   #await runloop.sleep_ms(150)
   #await armdown()
    await gyro_turn_left(20, fast=240, slow=110)  
    await settle()


runloop.run(main)





#async def arm_up():
    #motor.reset_relative_position(port.B, 0)
    #await motor.run_for_degrees(port.B, 200, 300)# 200° πάνω με speed 300


#async def arm_down():
    #await motor.run_for_degrees(port.B, 200, -300) # 200° κάτω (ανάποδα)
