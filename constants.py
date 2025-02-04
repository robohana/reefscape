# #!/usr/bin/env python3


import math
from wpimath.filter import SlewRateLimiter



#~ driving constants
class DrivingConstants:
    drivingSpeedLimiter = 1
    rotationSpeedLimiter = 1
    voltageLimiter = SlewRateLimiter(0.5)

class OIConstants:
    # Constants for the Operator Interface
    kDriverControllerPort = 0
    kOperatorControllerPort = 1
    deadzone = 0.2



class ModuleConstants:
    # Constants for the swerve module 
    kWheelRadius = 0.0508
    kEncoderResolution = 4096
    kModuleMaxAngularVelocity = math.pi
    kModuleMaxAngularAcceleration = math.tau
    kJoystickDeadband = 0.1  # Deadband for joystick inputs 
    kDistancePerRotation = math.tau / kEncoderResolution
    kpositionConversionFactor = math.tau * kWheelRadius / kEncoderResolution
    kPTurning = 0.2 #TODO: tune this value -JL on 2/3/25


class RobotConstants:


    # Drivetrain Constants
    
    kfrontLeftDriveID = 20
    kfrontLeftTurnID = 31
    frontLeftAbsoluteEncoderId = 3
    frontLeftDrivingMotorReversed = False
    frontLeftTurningMotorReversed = False
    frontLeftAbsoluteEncoderOffset = 1.57 #recalibrate this value
    frontLeftAbsoluteEncoderReversed = False #TODO: check if this is correct -JL on 2/3/25
    
    kfrontRightDriveID = 23
    kfrontRightTurnID = 22
    frontRightAbsoluteEncoderId = 0
    frontRightDrivingMotorReversed = True
    frontRightTurningMotorReversed = True
    frontRightAbsoluteEncoderOffset = 1.57 #recalibrate this value
    frontRightAbsoluteEncoderReversed = False #TODO: check if this is correct -JL on 2/3/25
    
    kbackLeftDriveID = 24
    kbackLeftTurnID = 30
    backLeftAbsoluteEncoderId = 2
    backLeftDrivingMotorReversed = False
    backLeftTurningMotorReversed = False
    backLeftAbsoluteEncoderOffset = 1.57 #recalibrate this value
    backLeftAbsoluteEncoderReversed = False #TODO: check if this is correct -JL on 2/3/25
    
    kbackRightDriveID = 21
    kbackRightTurnID = 29
    backRightAbsoluteEncoderId = 1
    backRightDrivingMotorReversed = True
    backRightTurningMotorReversed = False
    backRightAbsoluteEncoderOffset = 1.57 #recalibrate this value
    backRightAbsoluteEncoderReversed = False #TODO: check if this is correct -JL on 2/3/25
    




