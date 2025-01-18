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
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder, and turning encoder.

        :param driveMotorChannel: PWM output for the drive motor.
        :param turningMotorChannel: PWM output for the turning motor.
        :param turningEncoderChannel: Analog input for the turning encoder.
        :param invertDrive: Whether to invert the drive motor direction.
        :param invertTurn: Whether to invert the turning motor direction.
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

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(desiredState, encoderRotation)

        # Scale speed by cosine of angle error for smoother driving
        angleError = state.angle - encoderRotation
        state.speed *= angleError.cos()

        # Calculate the drive output from the drive PID controller
        driveOutput = self.drivePIDController.calculate(self.driveEncoder.getPosition(), state.speed)
        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the turning motor output from the turning PID controller
        turnOutput = self.turningPIDController.calculate(self.turningEncoder.getDistance(), state.angle.radians())
        turnFeedforward = self.turnFeedforward.calculate(self.turningPIDController.getSetpoint().velocity)

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)

