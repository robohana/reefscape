# Type: Subsystem
''' 
This file contains the DriveSubsystem class, which represents the drive subsystem of the robot.

'''
import math
import typing

import wpilib

from commands2 import Subsystem
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)

from constants import ModuleConstants, RobotConstants, DrivingConstants
import swerveutils
from .swervemodule import SwerveModule
from navx import AHRS
import navx

class DriveSubsystem(Subsystem):
    """Represents the drive subsystem of the robot. """

    def __init__(self) -> None:
        super().__init__()

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
        self.odometry = SwerveDrive4Odometry(
            DrivingConstants.kinematics,
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
    def periodic(self) -> None:
        # Update the odometry in the periodic block
        self.odometry.update(
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
    )
        
    def getPose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getPose()

    def resetOdometry(self, pose: Pose2d) -> None:
        """Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.

        """
        self.odometry.resetPosition(
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
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
            desiredStates, ModuleConstants.kmaxModuleVelocity
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(rl)
        self.backRight.setDesiredState(rr)

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


    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot.

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * (-1.0 if DrivingConstants.KGyroReversed else 1.0)

