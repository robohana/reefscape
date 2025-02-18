# #!/usr/bin/env python3


import math
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath import kinematics





#~ driving constants
class DrivingConstants:

    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds
    kMaxSpeedMetersPerSecond = 4.8
    kMaxAngularSpeed = math.tau  # radians per second

    kDirectionSlewRate = 1.2  # radians per second
    kMagnitudeSlewRate = 1.8  # percent per second (1 = 100%)
    kRotationalSlewRate = 2.0  # percent per second (1 = 100%)

    kTrackWidth = 0.584 #meters
     # Distance between centers of right and left wheels on robot
    kWheelBase = 0.584 #meters


    frontLeftLocation = Translation2d(kWheelBase / 2 , kTrackWidth / 2) 
    frontRightLocation = Translation2d(kWheelBase / 2 , -kTrackWidth / 2)
    backLeftLocation = Translation2d(-kWheelBase / 2 , kTrackWidth / 2)
    backRightLocation =  Translation2d(-kWheelBase / 2 , -kTrackWidth / 2)

    kinematics = kinematics.SwerveDrive4Kinematics(
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation)

    KGyroReversed = False
    kTeleopDriveMaxAccelerationMetersPerSecSquared = 3
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
    kWheelDiameter = 0.1016
    kEncoderResolution = 4096
    kJoystickDeadband = 0.1  # Deadband for joystick inputs 
    #kDistancePerRotation = math.tau / kEncoderResolution
    #kPositionConversionFactor = math.tau * kWheelRadius / kEncoderResolution
    
    ModuleMaxSpeedFTPerSec = 16.6 #ft/s from SwerveSpecialties website
    kmaxModuleVelocityMetersPerSec = 5.05968 #m/s #ModuleMaxSpeedFTPerSec / 3.281
    kmaxModuleAngularVelocity = kmaxModuleVelocityMetersPerSec /  kWheelRadius #rad/s #v max / wheel radius -JL on 2/6/25
    kmaxModuleAngularAcceleration = (kmaxModuleVelocityMetersPerSec * 2 / kWheelDiameter)/2 #rad/s^2

    kDriveMotorGearRatio = 1 / 6.12 #gear ratio from SwerveSpecialties website

    kTurningEncoderGearRatio = 1 / (150 / 7) #gear ratio from SwerveSpecialties website

    kDriveEncoderRot2Meter = kDriveMotorGearRatio * math.pi * kWheelDiameter
    kTurningEncoderRot2Rad = kTurningEncoderGearRatio * math.pi * 2

    kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60
    kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60



    kDriveP = 0.1
    kDriveI = 0
    kDriveD = 0

    kPTurning = 0.01
    kITurning = 0
    KDTurning = 0

    kDrivekS = 0.1
    kDrivekV = 1

    kTurningkS = 0.1
    kTurningkV = 0.227
    






class RobotConstants:


    # Drivetrain Constants
    
    kfrontLeftDriveID = 20
    kfrontLeftTurnID = 31
    frontLeftAbsoluteEncoderId = 3
    frontLeftDrivingMotorReversed = False
    frontLeftTurningMotorReversed = False
    frontLeftAbsoluteEncoderOffset = -math.pi / 2 #recalibrate this value
    frontLeftAbsoluteEncoderReversed = False #TODO: check if this is correct -JL on 2/3/25
    
    kfrontRightDriveID = 23
    kfrontRightTurnID = 22
    frontRightAbsoluteEncoderId = 0
    frontRightDrivingMotorReversed = True
    frontRightTurningMotorReversed = False
    frontRightAbsoluteEncoderOffset = 0  #recalibrate this value
    frontRightAbsoluteEncoderReversed = False #TODO: check if this is correct -JL on 2/3/25
    
    kbackLeftDriveID = 24
    kbackLeftTurnID = 30
    backLeftAbsoluteEncoderId = 2
    backLeftDrivingMotorReversed = False
    backLeftTurningMotorReversed = False
    backLeftAbsoluteEncoderOffset = math.pi #recalibrate this value
    backLeftAbsoluteEncoderReversed = False #TODO: check if this is correct -JL on 2/3/25
    
    kbackRightDriveID = 21
    kbackRightTurnID = 29
    backRightAbsoluteEncoderId = 1
    backRightDrivingMotorReversed = True
    backRightTurningMotorReversed = False
    backRightAbsoluteEncoderOffset = math.pi / 2 #recalibrate this value
    backRightAbsoluteEncoderReversed = False #TODO: check if this is correct -JL on 2/3/25

class AutoConstants:
    kMaxSpeedMetersPerSecond = 3
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi
    


