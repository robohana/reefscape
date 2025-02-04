import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
import wpimath.units
import rev
from constants import ModuleConstants




class SwerveModule:
    def __init__(self, driveMotorChannel: int, turningMotorChannel: int, absEncoderChannel: int, invertDrive: bool, invertTurn: bool, absouteEncoderOffset: float, absoluteEncoderReversed: bool) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder, and turning encoder. - JL

        :param driveMotorChannel: PWM output for the drive motor.
        :param turningMotorChannel: PWM output for the turning motor.
        :param turningEncoderChannel: Analog input for the turning encoder.
        :param invertDrive: Whether to invert the drive motor direction.
        :param invertTurn: Whether to invert the turning motor direction.
        """


        self.driveMotor = rev.SparkBase(driveMotorChannel, rev.SparkLowLevel.MotorType.kBrushless, rev.SparkLowLevel.SparkModel.kSparkMax)
        self.turningMotor = rev.SparkBase(turningMotorChannel, rev.SparkLowLevel.MotorType.kBrushless, rev.SparkLowLevel.SparkModel.kSparkMax)

        self.driveMotor.setInverted(invertDrive)
        self.turningMotor.setInverted(invertTurn)

        driveMotorConfig = rev.SparkMaxConfig()
        turnMotorConfig = rev.SparkMaxConfig()

        driveMotorConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        turnMotorConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)

        self.driveMotor.configure(driveMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
        self.turningMotor.configure(turnMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getEncoder()

        # Absolute Encoder
        # Absolute Encoders help "remember" the location of the module.
        # This is useful for when the robot is turned off and on again.
        self.absoluteEncoderOffsetRad = absouteEncoderOffset
        self.absoluteEncoderReversed = absoluteEncoderReversed
        
        self.absEncoder = wpilib.AnalogEncoder(absEncoderChannel)

        self.resetEncoders() 

        #TODO: Tune the drive and turn PID controllers - LC 1/30/25
        # Initialize PID controllers with initial gains - LC 1/30/25
        self.drivePIDController = wpimath.controller.PIDController(0.01, 0, 0)
        
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            ModuleConstants.kPTurning, 0, 0,  # Adjusted PID gains - JL
            wpimath.trajectory.TrapezoidProfile.Constraints(
                ModuleConstants.kModuleMaxAngularVelocity,
                ModuleConstants.kModuleMaxAngularAcceleration,
            ),
        )
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

        #TODO: Tune the drive and turn feed forward values - LC 1/30/25
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0.1, 1)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0.1, 0.227)

        #rev.EncoderConfig.positionConversionFactor(math.tau * kWheelRadius / kEncoderResolution)
        #self.turningEncoder.setDistancePerRotation(math.tau / kEncoderResolution)


    
    def getAbsoluteEncoderRad(self) -> float:
        """Returns the absolute encoder position in radians. - JL"""

        wpilib.SmartDashboard.putNumber("absEncoder", self.absEncoder.get())

        angle = self.absEncoder.get()
        angle *= 2 * math.pi #? convert to radians
        angle -= self.absoluteEncoderOffsetRad #? get acual location depending on the offset
        # TODO: calculate the actual offset, convert to rad, add here -JL on 2/3/25
        return angle * (-1 if self.absoluteEncoderReversed else 1)

    def resetEncoders(self):
        """Resets the drive and turning encoders to zero."""
        self.driveEncoder.setPosition(0)
        self.turningEncoder.setPosition(self.getAbsoluteEncoderRad())


    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module. - JL

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getPosition(), #TODO: Should this be getVelocity?? - JL on 2/3/25
            wpimath.geometry.Rotation2d(self.getAbsoluteEncoderRad()),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module. - JL"""
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getPosition() * ModuleConstants.kpositionConversionFactor,
            wpimath.geometry.Rotation2d(self.turningEncoder.getPosition() * ModuleConstants.kDistancePerRotation),
        )

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

        angle = wpimath.geometry.Rotation2d(self.turningEncoder.getPosition() * ModuleConstants.kDistancePerRotation)

        # Optimize the reference state to avoid spinning further than 90 degrees - LC 1/28/25
        desiredState.optimize(angle)

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving. - LC 1/28/25
        desiredState.cosineScale(angle)

        # Calculate the drive output from the drive PID controller. - LC 1/28/25
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getPosition() * ModuleConstants.kpositionConversionFactor, desiredState.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(desiredState.speed)

        # Calculate the turning motor output from the turning PID controller. - LC 1/28/25
        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.getPosition() * ModuleConstants.kDistancePerRotation, desiredState.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)


    def stop(self) -> None:
        """Stops the module."""
        self.driveMotor.set(0)
        self.turningMotor.set(0)

   
