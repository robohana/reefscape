
import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
import swervemodule
import rev
import navx
from constants import RobotConstants 
from navx import AHRS  # Import the AHRS class from the navX library
from wpimath.geometry import Pose2d


kMaxSpeed = 3.0  # 3 meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second

class Drivetrain:
    """Represents a swerve drive style drivetrain. - JL"""

    def __init__(self) -> None:


        self.frontLeftLocation = wpimath.geometry.Translation2d(0.381, 0.381)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.381, -0.381)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.381, 0.381)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.381, -0.381)

        self.frontLeft = swervemodule.SwerveModule(RobotConstants.kfrontLeftDriveID, 
                                                   RobotConstants.kfrontLeftTurnID, 
                                                   RobotConstants.frontLeftAbsoluteEncoderId, 
                                                   RobotConstants.frontLeftDrivingMotorReversed, 
                                                   RobotConstants.frontLeftTurningMotorReversed, 
                                                   RobotConstants.frontLeftAbsoluteEncoderOffset, 
                                                   RobotConstants.frontLeftAbsoluteEncoderReversed)
        
        self.frontRight = swervemodule.SwerveModule(RobotConstants.kfrontRightDriveID, 
                                                    RobotConstants.kfrontRightTurnID, 
                                                    RobotConstants.frontRightAbsoluteEncoderId, 
                                                    RobotConstants.frontRightDrivingMotorReversed, 
                                                    RobotConstants.frontRightTurningMotorReversed,
                                                    RobotConstants.frontRightAbsoluteEncoderOffset,
                                                    RobotConstants.frontRightAbsoluteEncoderReversed)
        
        self.backLeft = swervemodule.SwerveModule(RobotConstants.kbackLeftDriveID, 
                                                  RobotConstants.kbackLeftTurnID, 
                                                  RobotConstants.backLeftAbsoluteEncoderId, 
                                                  RobotConstants.backLeftDrivingMotorReversed, 
                                                  RobotConstants.backLeftTurningMotorReversed, 
                                                  RobotConstants.backLeftAbsoluteEncoderOffset,
                                                  RobotConstants.backLeftAbsoluteEncoderReversed)
        
        self.backRight = swervemodule.SwerveModule(RobotConstants.kbackRightDriveID, 
                                                   RobotConstants.kbackRightTurnID, 
                                                   RobotConstants.backRightAbsoluteEncoderId, 
                                                   RobotConstants.backRightDrivingMotorReversed, 
                                                   RobotConstants.backRightTurningMotorReversed, 
                                                   RobotConstants.backRightAbsoluteEncoderOffset,
                                                   RobotConstants.backRightAbsoluteEncoderReversed)

        #self.gyro = wpilib.ADXRS450_Gyro()
        self.gyro = AHRS(comType=navx._navx.AHRS.NavXComType.kMXP_SPI)

        self.gyro.reset()
        #update_rate = self.navx.getRate()
        # FR_isInverted = self.frontRight.driveMotor.getInverted()
        # wpilib.SmartDashboard.putBoolean(f"Front Right Inverted?", FR_isInverted)

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        # self.timer = wpilib.Timer()
        # self.timer.start()

        # wpilib.DataLogManager.start()
        # self.log = wpilib.DataLogManager.getLog()

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,  # Ensure field-relative mode is enabled by default - JL
        periodSeconds: float,
    ) -> None:
        """Method to drive the robot using joystick info. - JL

        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        if fieldRelative:
            # Transform the desired motion vector from robot-relative to field-relative - JL
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, self.gyro.getRotation2d()
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

        self.resetEncoders()

        # Telemetry - LC 1/30/25
        # self.showInverted()
        #self.dataManager()

    # def showInverted(self) -> None:
        # FR_isInverted = self.frontRight.driveMotor.getInverted() 
        # FL_isInverted = self.frontLeft.driveMotor.getInverted() 
        # BR_isInverted = self.backRight.driveMotor.getInverted() 
        # BL_isInverted = self.backLeft.driveMotor.getInverted()
        
        # Check if half of a second has passed - LC 1/30/25
        # Test 1, add TIMER to control data flow - LC 1/30/25
        # if self.timer.hasElapsed(0.5):
            # Reset the timer to count another 5 seconds, this is how you get it to loop every 5 seconds - LC 1/30/25
            # Removed right now because i only want to know at the start of the op which motors are inverted - LC 1/30/25
            #self.timer.reset()           
            # """Logs the motor invertion. Shows as a boolean value or green or red. - LC 1/30/25"""
            # Log if motor is inverted  for debugging
            #wpilib.SmartDashboard.putBoolean(f"Front Left Inverted?", FL_isInverted)
            #wpilib.SmartDashboard.putBoolean(f"Back Right Inverted?", BR_isInverted)
            #wpilib.SmartDashboard.putBoolean(f"Back Left Inverted?", BL_isInverted)
        

        # Test 3, see if this can use the DATALOGMANAGER to get out data instead of printing and causing the RIO CPU to spike
        # produced a very weird repeating "Front Left Inverted Data Log Manager: False" in my terminal after I would test the 
        # code - LC 1/30/25
        #wpilib.DataLogManager.log(f"Front Left Inverted Data Log Manager: {FL_isInverted}")


    def resetEncoders(self) -> None:
        """Resets the encoders for all swerve modules. - JL"""
        self.frontLeft.driveEncoder.setPosition(0)
        self.frontRight.driveEncoder.setPosition(0)
        self.backLeft.driveEncoder.setPosition(0)
        self.backRight.driveEncoder.setPosition(0)
        
        '''self.frontLeft.turningEncoder.reset()
        self.frontRight.turningEncoder.reset()
        self.backLeft.turningEncoder.reset()
        self.backRight.turningEncoder.reset()'''

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot. - JL"""
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

    def rotateWheels(self, angle: float) -> None:
        """Rotates all swerve modules to the specified angle. - JL"""
        desiredState = wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d.fromDegrees(angle))
        self.frontLeft.setDesiredState(desiredState)
        self.frontRight.setDesiredState(desiredState)
        self.backLeft.setDesiredState(desiredState)
        self.backRight.setDesiredState(desiredState)

    def autonomousRoutine(self) -> None:
        """Simple autonomous routine that rotates wheels 90 degrees to the right and then back to 0. - JL"""
        self.rotateWheels(90)

    # def dataManager(self) -> None :
    #     """This appends the data into the internal log, which is stored on the RoboRIO. This is all of the DS controls and 
    #     joystick data - LC 1/30/25"""
    #     # Record both DS control and joystick data - LC 1/30/25
    #     DriverStation.startDataLog(DataLogManager.getLog())
