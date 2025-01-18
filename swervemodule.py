import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
import rev

# Constants for the swerve module
kWheelRadius = 0.0508
kEncoderResolution = 4096
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau
kJoystickDeadband = 0.1  # Deadband for joystick inputs

class SwerveModule:
    def __init__(self, driveMotorChannel: int, turningMotorChannel: int, turningEncoderChannel: int) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel: PWM output for the drive motor.
        :param turningMotorChannel: PWM output for the turning motor.
        :param turningEncoderChannel: Analog input for the turning encoder.
        """
        self.driveMotor = rev.CANSparkMax(driveMotorChannel, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.turningMotor = rev.CANSparkMax(turningMotorChannel, rev.CANSparkLowLevel.MotorType.kBrushless)

        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = wpilib.AnalogEncoder(turningEncoderChannel)

        # Initialize PID controllers with initial gains
        self.drivePIDController = wpimath.controller.PIDController(0.01, 0, 0)
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            0.04, 0, 0,  # Adjusted PID gains
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )

        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0.1, 1)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0.1, 0.227)

        self.driveEncoder.setPositionConversionFactor(math.tau * kWheelRadius / kEncoderResolution)
        self.turningEncoder.setDistancePerRotation(math.tau / kEncoderResolution)

        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    #     # Calibrate the encoder to ensure it reads zero when the wheel is in the forward position
    #     self.calibrateEncoder()

    # def calibrateEncoder(self):
    #     """Calibrates the turning encoder to ensure it reads zero when the wheel is in the forward position."""
    #     # This function should be called when the wheel is physically set to the forward position
    #     self.turningEncoder.reset()

    
    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module."""
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getPosition(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getDistance()),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module."""
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getPosition(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getDistance()),
        )

    def setDesiredState(self, desiredState: wpimath.kinematics.SwerveModuleState) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """
        encoderRotation = wpimath.geometry.Rotation2d(self.turningEncoder.getDistance())

        # Log the current encoder rotation and desired state
        wpilib.SmartDashboard.putNumber("Current Encoder Rotation", encoderRotation.degrees())
        wpilib.SmartDashboard.putNumber("Desired Speed", desiredState.speed)
        wpilib.SmartDashboard.putNumber("Desired Angle", desiredState.angle.degrees())

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(desiredState, encoderRotation)

        # Log the desired state after optimization
        wpilib.SmartDashboard.putNumber("Optimized Speed", state.speed)
        wpilib.SmartDashboard.putNumber("Optimized Angle", state.angle.degrees())


        # Scale speed by cosine of angle error for smoother driving
        angleError = state.angle - encoderRotation
        wpilib.SmartDashboard.putNumber("Angle Error", angleError.degrees())
        state.speed *= angleError.cos()

        # Calculate the drive output from the drive PID controller
        driveOutput = self.drivePIDController.calculate(self.driveEncoder.getPosition(), state.speed)
        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the turning motor output from the turning PID controller
        turnOutput = self.turningPIDController.calculate(self.turningEncoder.getDistance(), state.angle.radians())
        turnFeedforward = self.turnFeedforward.calculate(self.turningPIDController.getSetpoint().velocity)

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)

        # Log the actual and desired angles for debugging
        wpilib.SmartDashboard.putNumber("Actual Angle", encoderRotation.degrees())
        wpilib.SmartDashboard.putNumber("Angle Error Cosine", angleError.cos())

        # Log the encoder values for debugging
        #self.logEncoderValues()

    # def logEncoderValues(self) -> None:
    #     """Logs the encoder values for debugging."""
    #     wpilib.SmartDashboard.putNumber("Drive Encoder Position", round(self.driveEncoder.getPosition(), 4))
    #     wpilib.SmartDashboard.putNumber("Turning Encoder Position", round(self.turningEncoder.getDistance(), 4))
