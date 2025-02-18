# Type: Subsystem
''' 
This file contains the DriveSubsystem class, which represents the drive subsystem of the robot.

'''
import math
import typing
import time
import struct
import threading
from wpilib import DriverStation

import wpilib

from commands2 import SubsystemBase
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition
)

from constants import ModuleConstants, RobotConstants, DrivingConstants
import swerveutils
from .swervemodule import SwerveModule
from navx import AHRS
import navx
from ntcore import NetworkTableInstance, StructPublisher, StructArrayPublisher
import time
import commands2




last_print_time = 0  # Global variable to track last print time





class DriveSubsystem(SubsystemBase):
    """Represents the drive subsystem of the robot. """

    def __init__(self) -> None:
        super().__init__()
        self.sd = wpilib.SmartDashboard


        self.nt = NetworkTableInstance.getDefault()
        self.swerveTable = self.nt.getTable("Swerve")

                #  **Create Struct Publishers for AdvantageScope**
        self.moduleStatesPublisher: StructArrayPublisher = (
            self.swerveTable.getStructArrayTopic("Drive/Modules/States", SwerveModuleState).publish()
        )
        self.desiredModuleStatesPublisher: StructArrayPublisher = (
            self.swerveTable.getStructArrayTopic("Drive/Modules/DesiredStates", SwerveModuleState).publish()
        )
        self.chassisSpeedsPublisher: StructPublisher = (
            self.swerveTable.getStructTopic("Drive/ChassisSpeeds", ChassisSpeeds).publish()
        )

        # Create swerve modules
        self.frontLeft = SwerveModule(RobotConstants.kfrontLeftDriveID, 
                                                   RobotConstants.kfrontLeftTurnID, 
                                                   RobotConstants.frontLeftAbsoluteEncoderId,  
                                                   RobotConstants.frontLeftAbsoluteEncoderOffset 
)
        
        self.frontRight = SwerveModule(RobotConstants.kfrontRightDriveID, 
                                                    RobotConstants.kfrontRightTurnID, 
                                                    RobotConstants.frontRightAbsoluteEncoderId, 
                                                    RobotConstants.frontRightAbsoluteEncoderOffset
)
        
        self.backLeft = SwerveModule(RobotConstants.kbackLeftDriveID, 
                                                  RobotConstants.kbackLeftTurnID, 
                                                  RobotConstants.backLeftAbsoluteEncoderId, 
                                                  RobotConstants.backLeftAbsoluteEncoderOffset
)
        
        self.backRight = SwerveModule(RobotConstants.kbackRightDriveID, 
                                                   RobotConstants.kbackRightTurnID, 
                                                   RobotConstants.backRightAbsoluteEncoderId, 
                                                   RobotConstants.backRightAbsoluteEncoderOffset
)

        #self.gyro = wpilib.ADXRS450_Gyro()
        self.gyro = AHRS(comType=navx._navx.AHRS.NavXComType.kMXP_SPI)

        # Slew rate filter variables for controlling lateral acceleration
        self.currentRotation = 0.0
        self.currentTranslationDir = 0.0
        self.currentTranslationMag = 0.0

        self.magLimiter = SlewRateLimiter(DrivingConstants.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(DrivingConstants.kRotationalSlewRate)
        self.prevTime = wpilib.Timer.getFPGATimestamp()

        # Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(DrivingConstants.kinematics, Rotation2d(0), self.getModulePositionsOld())

        thread = threading.Thread(target = self.zero_heading_after_delay)

        thread.start()

    def periodic(self) -> None:
        self.sd.putNumber("Gyro", self.getHeading())
        self.odometry.update(
                            self.getRotation2d(), 
                            (
                            (SwerveModulePosition(self.frontLeft.getDrivePosition(), Rotation2d(self.frontLeft.getAbsoluteEncoderRad())),
                            SwerveModulePosition(self.frontRight.getDrivePosition(), Rotation2d(self.frontRight.getAbsoluteEncoderRad())),
                            SwerveModulePosition(self.backLeft.getDrivePosition(), Rotation2d(self.backLeft.getAbsoluteEncoderRad())),
                            SwerveModulePosition(self.backRight.getDrivePosition(), Rotation2d(self.backRight.getAbsoluteEncoderRad())))
                            )
                            )
        
        self.sd.putString("Robot Odometer", str(self.getModulePositionsOld()))
        self.sd.putString("Robot Location, x", str(self.getPose().X()))
        self.sd.putString("Robot Location, y", str(self.getPose().Y()))
        self.sd.putString("Robot Location, rotation", str(self.getPose().rotation().degrees()))    



        # Publish swerve module states and chassis speeds for AdvantageScope
        #self.publishSwerveStates()



    def zero_heading_after_delay(self):
        try:
            time.sleep(1)
            self.gyro.reset()
        except Exception as e:
            pass
    
    
    
    def getPose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getPose()

    def resetOdometry(self, pose: Pose2d):
        self.odometry.resetPosition(
        self.getRotation2d(),
        (
            self.frontLeft.getSwerveModulePosition(),
            self.frontRight.getSwerveModulePosition(),
            self.backLeft.getSwerveModulePosition(),
            self.backRight.getSwerveModulePosition()
        ),
        pose,
        )

    def drive(
            self,
            xSpeed: float,
            ySpeed: float,
            rot: float,
            fieldRelative: bool,
            rateLimit: bool,
        ) -> None:
            """Method to drive the robot using joystick info.

            :param xSpeed:        Speed of the robot in the x direction (forward).
            :param ySpeed:        Speed of the robot in the y direction (strafe).
            :param rot:           Angular rate of the robot.
            :param fieldRelative: Whether the provided x and y speeds are relative to the
                                field.
            :param rateLimit:     Whether to enable rate limiting for smoother control.
            """

            print(f"Drive() Called - X:{xSpeed:.2f}, Y:{ySpeed:.2f}, Rot:{rot:.2f}")

            xSpeedCommanded = xSpeed
            ySpeedCommanded = ySpeed

            if rateLimit:
                # Convert XY to polar for rate limiting
                inputTranslationDir = math.atan2(ySpeed, xSpeed)
                inputTranslationMag = math.hypot(xSpeed, ySpeed)

                # Calculate the direction slew rate based on an estimate of the lateral acceleration
                if self.currentTranslationMag != 0.0:
                    directionSlewRate = abs(
                        DrivingConstants.kDirectionSlewRate / self.currentTranslationMag
                    )
                else:
                    directionSlewRate = 500.0
                    # some high number that means the slew rate is effectively instantaneous

                currentTime = wpilib.Timer.getFPGATimestamp()
                elapsedTime = currentTime - self.prevTime
                angleDif = swerveutils.angleDifference(
                    inputTranslationDir, self.currentTranslationDir
                )
                if angleDif < 0.45 * math.pi:
                    self.currentTranslationDir = swerveutils.stepTowardsCircular(
                        self.currentTranslationDir,
                        inputTranslationDir,
                        directionSlewRate * elapsedTime,
                    )
                    self.currentTranslationMag = self.magLimiter.calculate(
                        inputTranslationMag
                    )

                elif angleDif > 0.85 * math.pi:
                    # some small number to avoid floating-point errors with equality checking
                    # keep currentTranslationDir unchanged
                    if self.currentTranslationMag > 1e-4:
                        self.currentTranslationMag = self.magLimiter.calculate(0.0)
                    else:
                        self.currentTranslationDir = swerveutils.wrapAngle(
                            self.currentTranslationDir + math.pi
                        )
                        self.currentTranslationMag = self.magLimiter.calculate(
                            inputTranslationMag
                        )

                else:
                    self.currentTranslationDir = swerveutils.stepTowardsCircular(
                        self.currentTranslationDir,
                        inputTranslationDir,
                        directionSlewRate * elapsedTime,
                    )
                    self.currentTranslationMag = self.magLimiter.calculate(0.0)

                self.prevTime = currentTime

                xSpeedCommanded = self.currentTranslationMag * math.cos(
                    self.currentTranslationDir
                )
                ySpeedCommanded = self.currentTranslationMag * math.sin(
                    self.currentTranslationDir
                )
                self.currentRotation = self.rotLimiter.calculate(rot)

            else:
                self.currentRotation = rot

            # Convert the commanded speeds into the correct units for the drivetrain
            xSpeedDelivered = xSpeedCommanded * DrivingConstants.kMaxSpeedMetersPerSecond
            ySpeedDelivered = ySpeedCommanded * DrivingConstants.kMaxSpeedMetersPerSecond
            rotDelivered = self.currentRotation * DrivingConstants.kMaxAngularSpeed

            swerveModuleStates = DrivingConstants.kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(self.gyro.getAngle()),
                )
                if fieldRelative
                else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
            )
            fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
                swerveModuleStates, DrivingConstants.kMaxSpeedMetersPerSecond
            )
            self.frontLeft.setDesiredState(fl)
            self.frontRight.setDesiredState(fr)
            self.backLeft.setDesiredState(rl)
            self.backRight.setDesiredState(rr)

    def setModuleStates(
        self,
        desiredStates: typing.Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
    ) -> None:
        """Sets the swerve ModuleStates.

        :param desiredStates: The desired SwerveModule states.
        """
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, ModuleConstants.kmaxModuleVelocityMetersPerSec
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(rl)
        self.backRight.setDesiredState(rr)


    def getModulePositionsOld(self) -> tuple[SwerveModulePosition, SwerveModulePosition,SwerveModulePosition,SwerveModulePosition]:
        return (
                SwerveModulePosition(self.frontLeft.getDrivePosition(), Rotation2d(self.frontLeft.getAbsoluteEncoderRad())),
                SwerveModulePosition(self.frontRight.getDrivePosition(), Rotation2d(self.frontRight.getAbsoluteEncoderRad())),
                SwerveModulePosition(self.backLeft.getDrivePosition(), Rotation2d(self.backLeft.getAbsoluteEncoderRad())),
                SwerveModulePosition(self.backRight.getDrivePosition(), Rotation2d(self.backRight.getAbsoluteEncoderRad()))
                )
    def getModuleStates(self) -> tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
        return (
            SwerveModuleState(self.frontLeft.getDriveVelocity(), Rotation2d(self.frontLeft.getAbsoluteEncoderRad())),
                SwerveModuleState(self.frontRight.getDriveVelocity(), Rotation2d(self.frontRight.getAbsoluteEncoderRad())),
                SwerveModuleState(self.backLeft.getDriveVelocity(), Rotation2d(self.backLeft.getAbsoluteEncoderRad())),
                SwerveModuleState(self.backRight.getDriveVelocity(), Rotation2d(self.backRight.getAbsoluteEncoderRad()))
        )

    def getChassisSpeeds(self):
        return DrivingConstants.kinematics.toChassisSpeeds(self.getModuleStates())
    
    def driveChassisSpeeds(self, chassisSpeeds: ChassisSpeeds):
        self.setModuleStates(
            DrivingConstants.kinematics.toSwerveModuleStates(chassisSpeeds)
        )



    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.frontLeft.resetEncoders()
        self.backLeft.resetEncoders()
        self.frontRight.resetEncoders()
        self.backRight.resetEncoders()


    def zeroHeading(self) -> None:
        """Zeroes the heading of the robot."""
        self.gyro.reset()

    def getHeading(self) -> float:
        """Returns the heading of the robot.

        :returns: the robot's heading in degrees, from -180 to 180
        """
        return Rotation2d.fromDegrees(self.gyro.getAngle()).degrees()

    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.getHeading())
    
        #^used when we are able to adjust gyro with apriltags
    def setHeading(self, angle):
        self.gyro.reset()
        self.gyro.setAngleAdjustment(angle)


    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot.

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * (-1.0 if DrivingConstants.KGyroReversed else 1.0)
    

    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.getHeading())
        


    def stop(self) -> None:
        """Stops the module."""
        self.drive(0, 0, 0, False, False)


    def publishSwerveStates(self):
        """Publishes swerve module states and chassis speeds to AdvantageScope using correct struct format."""
        
        global last_print_time
        current_time = time.time()

        # Get all module states
        moduleStates = [
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.backLeft.getState(),
            self.backRight.getState(),
        ]

        #  **Publish `SwerveModuleState[]` as a structured array**
        self.moduleStatesPublisher.set(moduleStates)

        self.desiredModuleStatesPublisher.set([
            self.frontLeft.getDesiredState(),
            self.frontRight.getDesiredState(),
            self.backLeft.getDesiredState(),
            self.backRight.getDesiredState()
        ])

        #  **Publish `ChassisSpeeds` as a structured object**
        chassisSpeeds = DrivingConstants.kinematics.toChassisSpeeds(tuple(moduleStates))
        self.chassisSpeedsPublisher.set(chassisSpeeds)

        #  **Print only once per second**
        if current_time - last_print_time >= 1.0:
            print("Published Swerve Module States & Chassis Speeds (Struct Format)")
            print(f"Publishing Swerve Module States: {moduleStates}")
            print(f"Publishing Chassis Speeds: {chassisSpeeds}")
            last_print_time = current_time  # Update last print time

        #chassisSpeeds = DrivingConstants.kinematics.toChassisSpeeds(*moduleStates)



        #  Use setStruct to publish structured data for AdvantageScope
        self.swerveTable.getStructArrayTopic("ModuleStates", SwerveModuleState).publish().set(moduleStates)
        self.swerveTable.getStructTopic("ChassisSpeeds", ChassisSpeeds).publish().set(chassisSpeeds)


    def getModuleStates(self):
        """Returns the current states of all swerve modules."""
        return [
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.backLeft.getState(),
            self.backRight.getState(),
        ]







