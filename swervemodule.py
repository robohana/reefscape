import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
import wpimath.units
import rev

# Constants for the swerve module - JL
kWheelRadius = 0.0508
kEncoderResolution = 4096
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau
kJoystickDeadband = 0.1  # Deadband for joystick inputs - JL
kDistancePerRotation = math.tau / kEncoderResolution
kpositionConversionFactor = math.tau * kWheelRadius / kEncoderResolution


class SwerveModule:
    def __init__(self, driveMotorChannel: int, turningMotorChannel: int, turningEncoderChannel: int, setInverted: bool = False) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder, and turning encoder. - JL

        :param driveMotorChannel: PWM output for the drive motor.
        :param turningMotorChannel: PWM output for the turning motor.
        :param turningEncoderChannel: Analog input for the turning encoder.
        :param invertDrive: Whether to invert the drive motor direction.
        :param invertTurn: Whether to invert the turning motor direction.
        """
        self.driveMotor = rev.SparkBase(driveMotorChannel, rev.SparkLowLevel.MotorType.kBrushless, rev.SparkLowLevel.SparkModel.kSparkMax)
        self.turningMotor = rev.SparkBase(turningMotorChannel, rev.SparkLowLevel.MotorType.kBrushless, rev.SparkLowLevel.SparkModel.kSparkMax)

        self.driveMotor.setInverted(setInverted)

        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = wpilib.AnalogEncoder(turningEncoderChannel)

        #TODO: Tune the drive and turn PID controllers - LC 1/30/25
        # Initialize PID controllers with initial gains - LC 1/30/25
        self.drivePIDController = wpimath.controller.PIDController(0.01, 0, 0)
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            0.08, 0, 0,  # Adjusted PID gains - JL
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )

        #TODO: Tune the drive and turn feed forward values - LC 1/30/25
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0.1, 1)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0.1, 0.227)

        #rev.EncoderConfig.positionConversionFactor(math.tau * kWheelRadius / kEncoderResolution)
        #self.turningEncoder.setDistancePerRotation(math.tau / kEncoderResolution)

        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)


    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module. - JL

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getPosition(),
            wpimath.geometry.Rotation2d(self.turningEncoder.get() * kDistancePerRotation),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module. - JL"""
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getPosition() * kpositionConversionFactor,
            wpimath.geometry.Rotation2d(self.turningEncoder.get() * kDistancePerRotation),
        )

        #TODO: Rewrite this whole funtion - JL 1/28/25
    '''def setDesiredState(self, desiredState: wpimath.kinematics.SwerveModuleState) -> None:
        """Sets the desired state for the swerve module."""
    
        # Get current module rotation from encoder
        angle = wpimath.geometry.Rotation2d(self.turningEncoder.get() * DistancePerRotation)

        # Optimize the target state to prevent unnecessary rotation
        #optimizedState = wpimath.kinematics.SwerveModuleState.optimize(desiredState, currentRotation)
       
        # Calculate angle error for smooth turning (avoid small unnecessary turns)
        #angleError = optimizedState.angle.minus(currentRotation)
        #optimizedSpeed = angle.speed * angleError.cos()
        speed = (math.tau * kWheelRadius) / kEncoderResolution

        # Compute drive motor output using PID and feedforward control
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getPosition() * positionConversionFactor, desiredState.speed
        )
        driveFeedforward = self.driveFeedforward.calculate(speed)

        # Compute turn motor output using PID and feedforward control
        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.get() * DistancePerRotation, desiredState.angle.radians()
        )
        turnFeedforward = self.turnFeedforward.calculate(self.turningPIDController.getSetpoint().velocity)

        # Apply calculated voltages to motors
        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)'''

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
        ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        angle = wpimath.geometry.Rotation2d(self.turningEncoder.get() * kDistancePerRotation)

        # Optimize the reference state to avoid spinning further than 90 degrees - LC 1/28/25
        desiredState.optimize(angle)

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving. - LC 1/28/25
        desiredState.cosineScale(angle)

        # Calculate the drive output from the drive PID controller. - LC 1/28/25
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getPosition() * kpositionConversionFactor, desiredState.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(desiredState.speed)

        # Calculate the turning motor output from the turning PID controller. - LC 1/28/25
        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.get() * kDistancePerRotation, desiredState.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)

   
