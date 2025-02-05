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

        wpilib.SmartDashboard.putNumber("angle", angle)        
        return angle * (-1 if self.absoluteEncoderReversed else 1)

    def resetEncoders(self):
        """Resets the drive and turning encoders to zero."""
        self.driveEncoder.setPosition(0)
        self.turningEncoder.setPosition(self.getAbsoluteEncoderRad())


    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module. - JL"""
        speed = self.driveEncoder.getVelocity() #getVelocity here provides real-time speed of wheeel which is what we need not total distance traveled. - LC 2/4/25
        angle = wpimath.geometry.Rotation2d(    #! getVelocity returns the RPM of the motor -JL on 2/5/25
            self.getAbsoluteEncoderRad()
        )
        return wpimath.kinematics.SwerveModuleState(speed, angle)

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module. - JL"""
        position = self.driveEncoder.getPosition() * ModuleConstants.kPositionConversionFactor
        angle = wpimath.geometry.Rotation2d(
            self.turningEncoder.getPosition() * ModuleConstants.kDistancePerRotation
        )
        return wpimath.kinematics.SwerveModulePosition(position, angle)

    # def setDesiredState(self, desiredState: wpimath.kinematics.SwerveModuleState) -> None:
    #     """Sets the desired state for the module.

    #     :param desiredState: Desired state with speed and angle.
    #     """
    #     currentAngle = wpimath.geometry.Rotation2d(self.turningEncoder.getPosition() * ModuleConstants.kDistancePerRotation)

    #     # Optimize the desired state to minimize rotation
    #     optimizedState = wpimath.kinematics.SwerveModuleState.optimize(desiredState, self.getState().angle)

    #     # Optimize the reference state to avoid spinning further than 90 degrees - LC 1/28/25
    #     desiredState.optimize(currentAngle)

    #     # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired direction of travel that can occur when modules change directions. This results in smoother driving. - LC 1/28/25
    #     desiredState.cosineScale(currentAngle)

    #     # Calculate the drive output from the drive PID controller. - LC 1/28/25
    #     driveOutput = self.drivePIDController.calculate(optimizedState.speed)

    #     driveFeedforward = self.driveFeedforward.calculate(optimizedState.speed)

    #     # Calculate the turning motor output from the turning PID controller. - LC 1/28/25
    #     turnOutput = self.turningPIDController.calculate(currentAngle.radians(), optimizedState.angle.radians())

    #     turnFeedforward = self.turnFeedforward.calculate(self.turningPIDController.getSetpoint().velocity)

    #     self.driveMotor.setVoltage(driveOutput + driveFeedforward)
    #     self.turningMotor.setVoltage(turnOutput + turnFeedforward)


    def setDesiredState(self, desiredState: wpimath.kinematics.SwerveModuleState) -> None:
        """Sets the desired state for the swerve module."""
    
        # Optimize state to minimize rotation
        try:
            
            if desiredState.speed == 0.0: #Check if desiredState.speed == 0.0 before calling optimize(), If speed is 0, return desiredState directly (prevents None), Only call optimize() for nonzero speeds -JL on 2/5/25
                optimizedState = desiredState
            
            else:
                optimizedState = wpimath.kinematics.SwerveModuleState.optimize(
                    desiredState, self.getState().angle
                )
            print(f"Optimized state: {optimizedState}")
            print(f"current angle - desired state angle: {self.getState().angle - desiredState.angle}")
            print(f"Desired State: {desiredState}")
            print(f"Current Angle: {self.getState().angle}")

        except Exception as e:
            print(f"Optimization Failed...Setting state to desired state: {e}")
            optimizedState = desiredState  # Fallback if optimization fails
    
        # Drive motor output
        try:
            driveOutput = self.drivePIDController.calculate( #! error = optimizedState.speed - self.driveEncoder.getVelocity()
                self.driveEncoder.getVelocity(),               #! driveOutput = kP(0.01) * error
                optimizedState.speed
            )
            driveFeedforward = self.driveFeedforward.calculate(optimizedState.speed)
            self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        except Exception as e:
            #print(f"Drive error: {e}")
            self.driveMotor.setVoltage(0)
    
        # Turning motor output
        try:
            turnOutput = self.turningPIDController.calculate(
                self.turningEncoder.getPosition() * ModuleConstants.kDistancePerRotation,
                optimizedState.angle.radians()
            )
            turnFeedforward = self.turnFeedforward.calculate(
                self.turningPIDController.getSetpoint().velocity
            )
            self.turningMotor.setVoltage(turnOutput + turnFeedforward)
        except Exception as e:
            #print(f"Turn error: {e}")
            self.turningMotor.setVoltage(0)

    def stop(self) -> None:
        """Stops the module."""
        self.driveMotor.set(0)
        self.turningMotor.set(0)

   
