import commands2, constants, wpilib, navx, threading, time, math
from constants import OIConstants, DrivingConstants
from wpimath.filter import SlewRateLimiter
from commands2 import Command 
from wpilib import XboxController
from commands2.button import CommandXboxController
import wpimath
from subsystems.drivesubsystem import DriveSubsystem
from wpimath.kinematics import ChassisSpeeds

class SwerveJoystickCmd(Command):

    def __init__(self, robotDrive: DriveSubsystem, driverController:XboxController):
        super().__init__()
        self.robotDrive = robotDrive
        self.driverController = driverController
        self.addRequirements(self.robotDrive)
        # create Slew limiter
        self.xLimiter = SlewRateLimiter(DrivingConstants.kTeleDriveMaxAccelerationMetersPerSecSquared)
        self.yLimiter = SlewRateLimiter(DrivingConstants.kTeleDriveMaxAccelerationMetersPerSecSquared)
        self.zRotLimiter = SlewRateLimiter(DrivingConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond)

    def initialize(self):
        pass
    
    def execute(self):
        # these are multiplied by the drivingSpeedLimiter which limit the speed of the robot so it doesn't go too fast
        self.xSpeed = self.driverController.getLeftX() * DrivingConstants.drivingSpeedLimiter # self.drivingLimiter#* RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.ySpeed = self.driverController.getLeftY() * DrivingConstants.drivingSpeedLimiter # self.drivingLimiter #* RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.zRotation = self.driverController.getRightX() * -1 * DrivingConstants.rotationSpeedLimiter # self.drivingLimiter
        
        # 1. Get the joystick values and apply deadzone
        self.xSpeed = wpimath.applyDeadband(self.xSpeed, OIConstants.deadzone)
        self.ySpeed = wpimath.applyDeadband(self.ySpeed, OIConstants.deadzone)
        self.zRotation = wpimath.applyDeadband(self.zRotation, OIConstants.deadzone)
        

        # # 2. Add rateLimiter to smooth the joystick values
        self.xSpeed = self.xLimiter.calculate(self.xSpeed) * DrivingConstants.kTeleDriveMaxSpeedMetersPerSecond
        self.ySpeed = self.yLimiter.calculate(self.ySpeed) * DrivingConstants.kTeleDriveMaxSpeedMetersPerSecond
        self.zRotation = self.zRotLimiter.calculate(self.zRotation) * DrivingConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond #DrivingConstants.kMaxSpeedMetersPerSecond * DrivingConstants.kMaxSpeedMetersPerSecond / 0.418480
        #! Not sure why this is needed, for some reason without it, the robot rotates slower 

        # if self.fieldOriented:
        chasisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.xSpeed, self.ySpeed, self.zRotation, self.robotDrive.getRotation2d())
        
        # else:
        # If robotOrinted is desired (But what's the fun in that?)
        #     chasisSpeeds = ChassisSpeeds(self.xSpeed, self.ySpeed, self.zRotation)

        # 3. convert chasis speeds to module states
        moduleStates = DrivingConstants.kinematics.toSwerveModuleStates(chasisSpeeds)




        self.robotDrive.setModuleStates(moduleStates)
        

    def end(self, interrupted: bool):
        self.robotDrive.stop()

    def isFinished(self):
        return False
