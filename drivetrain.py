import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
import swervemodule
from navx import AHRS  # Import the AHRS class from the navX library

kMaxSpeed = 3.0  # 3 meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second

class Drivetrain:
    """Represents a swerve drive style drivetrain."""

    def __init__(self) -> None:
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.381, 0.381)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.381, -0.381)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.381, 0.381)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.381, -0.381)

        self.frontLeft = swervemodule.SwerveModule(20, 31, 3)
        self.frontRight = swervemodule.SwerveModule(23, 22, 0)
        self.backLeft = swervemodule.SwerveModule(24, 30, 2)
        self.backRight = swervemodule.SwerveModule(21, 29, 1)

        self.gyro = AHRS(wpilib.SPI.Port.kMXP)  # Initialize the navX gyro on the MXP port

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            wpimath.geometry.Rotation2d.fromDegrees(self.gyro.getAngle()),  # Get the angle from the navX gyro
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        self.gyro.reset()  # Reset the navX gyro

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,  # Ensure field-relative mode is enabled by default
        periodSeconds: float,
    ) -> None:
        """Method to drive the robot using joystick info.

        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        if fieldRelative:
            # Transform the desired motion vector from robot-relative to field-relative
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, wpimath.geometry.Rotation2d.fromDegrees(self.gyro.getAngle())
            )
        else:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)

        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(chassisSpeeds, periodSeconds)
        )

        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )

        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

        self.logEncoderValues()

    def logEncoderValues(self) -> None:
        """Logs the encoder values for each swerve module."""
        # Log encoder values for debugging
        # wpilib.SmartDashboard.putNumber("Front Left Turning Encoder", self.frontLeft.turningEncoder.getDistance()*10000)
        # wpilib.SmartDashboard.putNumber("Front Left Turning Encoder num of turns", self.frontLeft.turningEncoder.get())
        # wpilib.SmartDashboard.putNumber("Front Right Turning Encoder", self.frontRight.turningEncoder.getDistance()*10000)
        # wpilib.SmartDashboard.putNumber("Front Right Turning Encoder num of turns", self.frontRight.turningEncoder.get())
        # wpilib.SmartDashboard.putNumber("Back Left Turning Encoder", self.backLeft.turningEncoder.getDistance()*10000)
        # wpilib.SmartDashboard.putNumber("Back Left Turning Encoder num of turns", self.backLeft.turningEncoder.get())
        # wpilib.SmartDashboard.putNumber("Back Right Turning Encoder", self.backRight.turningEncoder.getDistance()*10000)
        # wpilib.SmartDashboard.putNumber("Back Right Turning Encoder num of turns", self.backRight.turningEncoder.get())

    def resetEncoders(self) -> None:
        """Resets the encoders for all swerve modules."""
        self.frontLeft.driveEncoder.setPosition(0)
        self.frontRight.driveEncoder.setPosition(0)
        self.backLeft.driveEncoder.setPosition(0)
        self.backRight.driveEncoder.setPosition(0)
        
        self.frontLeft.turningEncoder.reset()
        self.frontRight.turningEncoder.reset()
        self.backLeft.turningEncoder.reset()
        self.backRight.turningEncoder.reset()

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            wpimath.geometry.Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

    def rotateWheels(self, angle: float) -> None:
        """Rotates all swerve modules to the specified angle."""
        desiredState = wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d.fromDegrees(angle))
        self.frontLeft.setDesiredState(desiredState)
        self.frontRight.setDesiredState(desiredState)
        self.backLeft.setDesiredState(desiredState)
        self.backRight.setDesiredState(desiredState)

    def autonomousRoutine(self) -> None:
        """Simple autonomous routine that rotates wheels 90 degrees to the right and then back to 0."""
        self.rotateWheels(90)