#region VEXcode Generated Robot Configuration
from vex import *
import urandom # type: ignore
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
claire = DigitalOut(brain.three_wire_port.h)


# wait for rotation sensor to fully initialize
wait(30, MSEC)


# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration

#region Helper classes and function
import time
class RealSmartDrive:
    TIME_STEP_SEC = 0.005 # in seconds
    TURN_ANGLE_TOLERANCE = 0.2 # Degrees
    MIN_DRIVE_SPEED_PCT = 20
    MIN_TURN_SPEED_PCT = 2
    SLOW_TURN_ANGLE_DEG = 10
    SMOOTH_SPEED_DISTANCE_MM = 100
    MAX_VOLTAGE = 12
    # The minimum voltage to move the robot. This value could be different for each robot
    # based on the robot weight and friction.
    # Too low, the robot will not move and get stuck in the PID loop forerver.
    # Too high, the robot will jerk when close to target.
    MIN_VOLTAGE = 2.5

    def __init__(self, left_motor, right_motor, inertial, wheel_travel, track_width, wheel_base, external_gear_ratio, inertial_rotation_ratio, p_const, d_const):
        """RealSmartDrive mirrors the VEX SmartDrive, but it drives with PID corrections."""
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.inertial = inertial
        self.wheel_travel = wheel_travel
        self.track_width = track_width
        self.wheel_base = wheel_base
        self.external_gear_ratio = external_gear_ratio
        self.inertial_rotation_ratio = inertial_rotation_ratio
        self.p_const = p_const
        self.d_const = d_const
        # for screen output sampling
        self.step_count = 0
        # This track the heading of the robot at all times
        # It should equal to the inertial rotation reading when the robot is going stright or at rest
        # It should be the target rotation when the robot is turning in place
        self.target_rotation = 0

    def _sampleScreenPrint(self, message):
        """To print every 5th message, for debugging purposesly only"""
        self.step_count += 1
        if self.step_count % 20 == 0:
            brain.screen.print(message)
            brain.screen.next_row()

    def turn_for(self, direction, turn_value, turn_unit=DEGREES, wait=True):
        """Turn for a speified angle."""
        if direction not in [LEFT, RIGHT]:
            raise ValueError("direction can only be either LEFT or RIGHT")
        if wait != True:
            raise ValueError("Wait can only be True. No-wait is not yet implemented")
        
        left_motor.reset_position()
        right_motor.reset_position()
        rotation_error = - turn_value if direction == LEFT else turn_value
        self.target_rotation = self.target_rotation + rotation_error

        TURN_KP = 0.25
        TURN_KD = 2.5
        DRIFT_KP = 0.05

        # Variables for Derivative (D) term
        prev_error = 0
        settle_count = 0 # to prevent fluctuation around target
        
        brain.screen.print("Start turning to " + str(self.target_rotation) + " deg")
        brain.screen.next_row()

        while True:
            error = self.target_rotation - inertial.rotation(DEGREES)
            if abs(error) < self.TURN_ANGLE_TOLERANCE:
                settle_count += 1
                if settle_count > 2:
                    break
            
            # The derivative is the speed of error change. i.e how much change over one loop
            derivative = error - prev_error
            turn_power = error * TURN_KP + derivative * TURN_KD
            prev_error = error
            
            # DRIFT PID to ensure the left and right motors move equally when turning
            # If turning in place, Left moves (+) and Right moves (-).
            # Ideally: Left_Pos + Right_Pos = 0
            left_pos = left_motor.position(DEGREES)
            right_pos = right_motor.position(DEGREES)
            drift_error = left_pos + right_pos
            drift_correction = drift_error * DRIFT_KP
            
            left_volts = turn_power - drift_correction
            right_volts = -turn_power - drift_correction

            # Adjust voltages to be within min and max voltages, using absolute values
            # to avoid repeating the code, cuz I am lazy.
            abs_left_volts = self.MIN_VOLTAGE + math.fabs(left_volts) / self.MAX_VOLTAGE * (self.MAX_VOLTAGE - self.MIN_VOLTAGE)
            abs_left_volts = abs_left_volts if abs_left_volts <= self.MAX_VOLTAGE else self.MAX_VOLTAGE
            abs_right_volts = self.MIN_VOLTAGE + math.fabs(right_volts) / self.MAX_VOLTAGE * (self.MAX_VOLTAGE - self.MIN_VOLTAGE)
            abs_right_volts = abs_right_volts if abs_right_volts <= self.MAX_VOLTAGE else self.MAX_VOLTAGE

            # apply the sign back to the adjusted absolute voltages
            left_volts = math.copysign(abs_left_volts, left_volts)
            right_volts = math.copysign(abs_right_volts, right_volts)

            left_motor.spin(FORWARD, left_volts, VOLT) # type: ignore
            right_motor.spin(FORWARD, right_volts, VOLT) # type: ignore
            # self._sampleScreenPrint("E:" + str(error) + " D:" + str(derivative) + " L:" + str(left_volts) + " R:" + str(right_volts))
            time.sleep(RealSmartDrive.TIME_STEP_SEC)

        left_motor.stop(BRAKE)
        right_motor.stop(BRAKE)
        brain.screen.print("End turning at " + str(inertial.rotation()) + " deg")
        brain.screen.next_row()
        brain.screen.print("L Rotation:" + str(left_motor.position(DEGREES)) + " R Rotation:" + str(right_motor.position(DEGREES)))
        brain.screen.next_row()
        
        
    def drive_for(self, direction, distance, dis_unit=MM, speed_pct=100, speed_unit=PERCENT, wait=True):
        """Drive the robot by a specific distance."""
        if dis_unit != MM:
            raise ValueError("Distance unit must be MM in RealSmartDrive.")
        if speed_pct <=0 or speed_pct > 100:
            raise ValueError("speed must be between > 0 but <= 100")
        if speed_unit != PERCENT:
            raise ValueError("speed_unit must be PERCENT")
        if wait != True:
            raise ValueError("Wait can only be True. No-wait is not yet implemented")

        speed_volt = RealSmartDrive.MAX_VOLTAGE * speed_pct / 100
        left_speed_volt =  speed_volt
        right_speed_volt = speed_volt
        left_motor.set_stopping(COAST)
        right_motor.set_stopping(COAST)
        left_motor.reset_position()
        right_motor.reset_position()
        previous_rotation_error = self.inertial.rotation(DEGREES) - self.target_rotation
        traveled_dis = 0 
        smooth_distance = RealSmartDrive.SMOOTH_SPEED_DISTANCE_MM * speed_pct / 100
        while(traveled_dis < distance):
            # The P value in PID is based on the degree deviation off the rotation, limited to 45 degrees
            rotation_error = self.inertial.rotation(DEGREES) - self.target_rotation
            porportion = self.p_const * rotation_error
            porportion = -45 if porportion < -45 else (45 if porportion > 45 else porportion)

            # The D value in PID is the speed (degress/s) rotation angle changes over last the caclulation cycle, limited to 45 degrees
            gyro_rate_dps = (rotation_error - previous_rotation_error) / RealSmartDrive.TIME_STEP_SEC
            previous_rotation_error = rotation_error
            derivative = self.d_const * gyro_rate_dps * RealSmartDrive.TIME_STEP_SEC
            derivative = -45 if derivative < -45 else (45 if derivative > 45 else derivative)

            # slow down the faster side.
            speed_adj = (porportion + derivative) / 45
            if direction == REVERSE:
                speed_adj = -speed_adj
            left_speed_volt = speed_volt - (0 if speed_adj <= 0 else speed_volt * speed_adj)
            right_speed_volt = speed_volt - (0 if speed_adj >= 0 else speed_volt * -speed_adj)
            self._sampleScreenPrint(str(left_speed_volt) + ":" + str(right_speed_volt))

            if traveled_dis / distance < 0.5:
                smooth_speed_factor = 1 if traveled_dis > smooth_distance else traveled_dis / smooth_distance
            else:
                smooth_speed_factor = 1 if (distance - traveled_dis) > smooth_distance else (distance - traveled_dis) / smooth_distance
            smooth_speed_factor = RealSmartDrive.MIN_DRIVE_SPEED_PCT/100 + (100 - RealSmartDrive.MIN_DRIVE_SPEED_PCT)/100 * smooth_speed_factor

            left_motor.spin(direction, left_speed_volt * smooth_speed_factor, VOLT) # type: ignore
            right_motor.spin(direction, right_speed_volt * smooth_speed_factor, VOLT) # type: ignore
            time.sleep(RealSmartDrive.TIME_STEP_SEC)
            traveled_dis = math.fabs(self.wheel_travel * self.external_gear_ratio * (left_motor.position(TURNS) + right_motor.position(TURNS)) / 2)

        left_motor.stop(BRAKE)
        right_motor.stop(BRAKE)
        rotation_error = self.inertial.rotation(DEGREES)
        brain.screen.print("DER:" + str(rotation_error))
        brain.screen.next_row()
    
    def set_stopping(self, mode=COAST):
        self.left_motor.set_stopping(mode)
        self.right_motor.set_stopping(mode)
        
#endregion Helper classes and function

#
# Library imports
# ------------------------------------------
ROBOT_NAME = "BACKUP" # "HELPLESS"

if ROBOT_NAME == "HELPLESS":
    WHEEL_TRAVEL = 260 # backup robot, the MVP
    EXT_GEAR_RATIO = 1.19 # Drive for a fixed length and calculate this value competions 1.19 testbot 1.7
    TRACK_WIDTH = 270 #mm
    WHEEL_BASE = 300 #mm
if ROBOT_NAME == "BACKUP":
    WHEEL_TRAVEL = 260 # backup robot, the MVP
    EXT_GEAR_RATIO = 1.7 # Drive for a fixed length and calculate this value competions 1.19 testbot 1.7
    TRACK_WIDTH = 290.5 #mm
    WHEEL_BASE = 165 #mm

# Robot configuration code
motor_group_1_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
motor_group_1_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)
left_motor = MotorGroup(motor_group_1_motor_a, motor_group_1_motor_b)
motor_group_2_motor_a = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
motor_group_2_motor_b = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
right_motor = MotorGroup(motor_group_2_motor_a, motor_group_2_motor_b)
loader_piston = DigitalOut(brain.three_wire_port.h)
inertial = Inertial(Ports.PORT12)
IntakeMotor = Motor(Ports.PORT6, GearSetting.RATIO_18_1, False)
HoodMotor = Motor(Ports.PORT7, GearSetting.RATIO_18_1, False)

#smart_drive = SmartDrive(left_motor, right_motor, inertial, WHEEL_TRAVEL, TRACK_WIDTH, WHEEL_BASE, MM, EXT_GEAR_RATIO)
smart_drive = RealSmartDrive(left_motor, right_motor, inertial, WHEEL_TRAVEL, TRACK_WIDTH, WHEEL_BASE, EXT_GEAR_RATIO, 0.9925, 5, 50)
controller_1 = Controller(PRIMARY)

# global variables
# NOTE: Use ROBOT_INITIALIZED to allow movement. Calibration time is hidden when connected to field, but we need to prevent robot
#  from moving if we just do Program->Run on the controller
ROBOT_INITIALIZED = False

# ------------------------------------------
#
# Section 2:  Functions that define how the different joystick
#       buttons should work
#
# ------------------------------------------


#claire.set is the piston code and it does piston thingys
def pistondown():
    claire.set(False)

def pistonup():
    claire.set(True)


# button stuff
isIntakeMotorspinningforward = False
isHoodMotorspinningforward = False
isIntakeMotorspinningbackward = False
isHoodMotorspinning = False
ispistondown = False

IntakeMotor.set_velocity(90, PERCENT)
HoodMotor.set_velocity(90, PERCENT)

def whenControllerUpPressed():
   brain.screen.print("UpButtonPressed")
   brain.screen.next_row()

def whenControllerXPressed():
    brain.screen.print("XPressed")
    brain.screen.next_row()
  
def whenControllerBPressed():
   brain.screen.print("BPressed")
   brain.screen.next_row()

def whenControllerAPressed():
  brain.screen.print("APressed")
  brain.screen.next_row()

def whenControllerYPressed():  
   brain.screen.print("YPressed")
   brain.screen.next_row()
 
#hi slay yas queen :p


#spin intake backward
def whenControllerL1Pressed():
    brain.screen.clear_screen()
    global isIntakeMotorspinningbackward
    if isIntakeMotorspinningbackward == False:
        IntakeMotor.spin(REVERSE)
        isIntakeMotorspinningbackward = True
        isIntakeMotorspinningforward = False
    else:
        IntakeMotor.stop()
        isIntakeMotorspinningbackward = False
    brain.screen.next_row()

#spin intake forward
def whenControllerL2Pressed():
    brain.screen.clear_screen()
    global isIntakeMotorspinningforward
    if isIntakeMotorspinningforward == False:
        IntakeMotor.spin(FORWARD)
        isIntakeMotorspinningforward = True
        isIntakeMotorspinningbackward = False
    else:
        IntakeMotor.stop()
        isIntakeMotorspinningforward = False
    brain.screen.next_row()

#loader bar
def whenControllerR1Pressed():
    global ispistondown
    if ispistondown == False:
        pistondown()
        ispistondown = True
    else:
        pistonup()
        ispistondown = False

#spin hood motor
def whenControllerR2Pressed():
    global isHoodMotorspinning
    if isHoodMotorspinning == False:
        HoodMotor.spin(FORWARD)
        isHoodMotorspinning = True
    else:
        HoodMotor.stop()
        isHoodMotorspinning = False

    brain.screen.next_row()


 
# ------------------------------------------
#
# Section 3:  Function that defines what the robot should do during
#       the autonomous round, and function that defines how
#       the driver-controlled portion should work
#
# ------------------------------------------

def pre_autonomous():
    global ROBOT_INITIALIZED

    # calibrate inertial and wait for completion - takes around 2 seconds
    # IMPORTANT: Robot must be stationary on a flat surface while this runs. Do not touch robot during calibration
    if inertial.installed():
        inertial.calibrate()
        while inertial.is_calibrating():
            wait(2, SECONDS)
        inertial.reset_rotation()
    ROBOT_INITIALIZED = True

def autonomous():
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)
        
    # for 60 in expression_list:
    #     pass
    def right_basic():
        pistonup()
        smart_drive.drive_for(FORWARD, 850, MM, wait=True)
        smart_drive.turn_for(RIGHT, 90, DEGREES, wait=True)
        pistondown()
        IntakeMotor.spin_for(FORWARD, 100, TURNS, wait =False)
        smart_drive.drive_for(FORWARD, 300, MM, 100, PERCENT, wait=True)
        smart_drive.drive_for(REVERSE, 700, MM, 100, PERCENT, wait=True)
        pistonup()
        HoodMotor.spin_for(FORWARD, 5, TURNS, wait=True)
    
    
    def left_basic():
        smart_drive.drive_for(FORWARD, 200, MM, 100, PERCENT, wait=True) 

    
    right_basic()

brain.screen.print("heyy")


def user_control():
    brain.screen.print("HELP is so awesomesauce bro skibidi slay 677!")
    brain.screen.next_row()

    # Install callbacks for buttons.  This is where you map the joystick
    # buttons to different functions that you defined above.
    controller_1.buttonL1.pressed(whenControllerL1Pressed)
    controller_1.buttonL2.pressed(whenControllerL2Pressed)
    controller_1.buttonR1.pressed(whenControllerR1Pressed)
    controller_1.buttonR2.pressed(whenControllerR2Pressed)
    controller_1.buttonUp.pressed(whenControllerUpPressed)
    controller_1.buttonX.pressed(whenControllerXPressed)
    controller_1.buttonB.pressed(whenControllerBPressed)
    controller_1.buttonA.pressed(whenControllerAPressed)
    controller_1.buttonY.pressed(whenControllerYPressed)

    SMOOTH_THRESHOLD = 20 # Only adjust speed if speed delta exceeds this value
    SMOOTH_STEP = 15 # 250ms/20ms*8 = 100. i.e. goes from 0 to 100 in 0.5 sec
    def smooth_speed(user_speed):
        current_speed = (left_motor.velocity(PERCENT) + right_motor.velocity(PERCENT)) / 2
        if math.fabs(user_speed) < SMOOTH_THRESHOLD:
            return user_speed
        # acceleration forward, or decelerate while going backward
        if user_speed - current_speed > SMOOTH_THRESHOLD:
            return current_speed + SMOOTH_STEP
        # accelerate backward or decelerate while going forward
        if user_speed - current_speed < -SMOOTH_THRESHOLD:
            return current_speed - SMOOTH_STEP
        return user_speed

    # DRIVE_MODE can be any of these['raw', 'step', 'exp']
    # raw: no joystick sugar coating. Recommended for the best of the best drivers
    # step: Just slam it pedal to metal and the code smooth accelerate and decelerations. Good for grandmas
    # exp: Turns joystick output to a exponetial curve. Good driving skills needed.
    DRIVE_MODE = 'step'

    while True: 
        # The axis values ranges from -100 to +100. Farthest position left or down = -100
        raw_joytick_updown_value = controller_1.axis3.position() - controller_1.axis2.position()
        updown_value = raw_joytick_updown_value if DRIVE_MODE == 'raw' else smooth_speed(raw_joytick_updown_value) if DRIVE_MODE == 'step' else raw_joytick_updown_value ** 3 / 100**2
        leftright_value = (controller_1.axis4.position() + controller_1.axis1.position()) / 4 # don't need to turn faster than 25% speed
        leftright_value = 0 if math.fabs(leftright_value) < 10 else leftright_value
        left_motor.set_velocity(updown_value + leftright_value, PERCENT)
        right_motor.set_velocity(updown_value - leftright_value, PERCENT)
        left_motor.spin(FORWARD)
        right_motor.spin(FORWARD)

        wait(20, MSEC)

# ------------------------------------------
#
# Section 4:  This is the “main” portion of the code that gets
#       executed in the very beginning.  Here you specify
#       which functions to run during the autonomous &
#           driver controlled portions of the competition
#
# ------------------------------------------

# create competition instance
# comp = Competition(user_control, autonomous)
pre_autonomous()

def spin_robot_no_pid():
    """Spin the robot in place without PID control.
    
    This is a good test to see how much the robot drifts when turning without any correction.
    If left and right motors are perfectly matched, the robot should turn in place without any drift.
    We certainly want to tune to the hardware to minimize the drift without the help of PID.

    Factors to build a Zero-Radius-Turn robot:
    1. Make sure left and right motors are matched.
    2. Make sure the friction on both sides are similar.
    3. Make sure the weight distribution is even on both sides.
    4. Make sure the wheels are aligned properly.
    """
    left_motor.reset_position()
    right_motor.reset_position()

    TURN_VOLTAGE = 12

    start_time = time.time()
    run_time = 5 # seconds

    while True:
        if time.time() - start_time > run_time:
            break

        left_motor.spin(FORWARD, TURN_VOLTAGE, VOLT) # type: ignore
        right_motor.spin(FORWARD, -TURN_VOLTAGE, VOLT) # type: ignore
        time.sleep(RealSmartDrive.TIME_STEP_SEC)
    
    left_motor.stop(BRAKE)
    right_motor.stop(BRAKE)
    brain.screen.print("L Rotation:" + str(left_motor.position(DEGREES)) + " R Rotation:" + str(right_motor.position(DEGREES)))
    brain.screen.next_row()

def spin_robot_pid():
    """Spin the robot in place with a simple DRIFT Porportional control.
    
    In theory this should help minimize drift. But in the real world, the wheels
    could slip and render the Porportional control ineffective. i.e. At the end
    both wheels travelled the same distance (but oposite direction), but the rotational
    center of the robot would still drift.
    """
    # try to tune this.
    DRIFT_KP = 0.5
    MAX_VOLTAGE = 5
    TURN_VOLTAGE = 3
    MIN_VOLTAGE = 2.5
    left_motor.reset_position()
    right_motor.reset_position()

    start_time = time.time()
    run_time = 5 # seconds

    while True:
        if time.time() - start_time > run_time:
            break

         # DRIFT PID to ensure the left and right motors move equally when turning
        left_pos = left_motor.position(DEGREES)
        right_pos = right_motor.position(DEGREES)
        drift_error = left_pos + right_pos
        drift_correction = drift_error * DRIFT_KP
            
        left_volts = TURN_VOLTAGE - drift_correction
        right_volts = -TURN_VOLTAGE - drift_correction

        abs_left_volts = MIN_VOLTAGE + math.fabs(left_volts) / MAX_VOLTAGE * (MAX_VOLTAGE - MIN_VOLTAGE)
        abs_left_volts = abs_left_volts if abs_left_volts <= MAX_VOLTAGE else MAX_VOLTAGE
        abs_right_volts = MIN_VOLTAGE + math.fabs(right_volts) / MAX_VOLTAGE * (MAX_VOLTAGE - MIN_VOLTAGE)
        abs_right_volts = abs_right_volts if abs_right_volts <= MAX_VOLTAGE else MAX_VOLTAGE

        left_volts = math.copysign(abs_left_volts, left_volts)
        right_volts = math.copysign(abs_right_volts, right_volts)

        left_motor.spin(FORWARD, left_volts, VOLT) # type: ignore
        right_motor.spin(FORWARD, right_volts, VOLT) # type: ignore
        # self._sampleScreenPrint("E:" + str(error) + " D:" + str(derivative) + " L:" + str(left_volts) + " R:" + str(right_volts))
        time.sleep(RealSmartDrive.TIME_STEP_SEC)
    
    left_motor.stop(BRAKE)
    right_motor.stop(BRAKE)
    brain.screen.print("L Rotation:" + str(left_motor.position(DEGREES)) + " R Rotation:" + str(right_motor.position(DEGREES)))
    brain.screen.next_row()

def test_min_voltage_to_spin_robot():
    # tested and found that 2.5V is the minium to move the robot
    left_motor.spin(FORWARD, 2.5, VOLT) # type: ignore
    right_motor.spin(FORWARD, -2.5, VOLT) # type: ignore
    wait(10, SECONDS)
    left_motor.stop()
    right_motor.stop()


def demo():
    smart_drive.turn_for(RIGHT, 90, DEGREES, wait=True)
    smart_drive.turn_for(RIGHT, 90, DEGREES, wait=True)
    smart_drive.turn_for(RIGHT, 90, DEGREES, wait=True)
    smart_drive.turn_for(RIGHT, 90, DEGREES, wait=True)

    wait(2, SECONDS)

    smart_drive.turn_for(LEFT, 90, DEGREES, wait=True)
    smart_drive.turn_for(LEFT, 90, DEGREES, wait=True)
    smart_drive.turn_for(LEFT, 90, DEGREES, wait=True)
    smart_drive.turn_for(LEFT, 90, DEGREES, wait=True)
 
demo()
# spin_robot_pid()
# spin_robot_no_pid()
