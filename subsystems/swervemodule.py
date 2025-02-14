import math
import wpilib
import time
import threading
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
import wpimath.units
from wpimath.trajectory import TrajectoryConfig
from wpilib import AnalogEncoder
import commands2

from wpimath.controller import PIDController, ProfiledPIDController, SimpleMotorFeedforwardMeters, SimpleMotorFeedforwardRadians
from constants import ModuleConstants
from rev import SparkMax, SparkMaxConfig, SparkBaseConfig, SparkBase
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from ntcore import NetworkTableInstance
from commands2 import CommandScheduler

class SwerveModule(commands2.SubsystemBase):
    def __init__(self, driveMotorChannel: int, turningMotorChannel: int, absEncoderChannel: int, absouteEncoderOffset: float) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder, and turning encoder."""
        super().__init__()
        # Force NetworkTables to start
        self.nt = NetworkTableInstance.getDefault()
        self.debugTable = self.nt.getTable("SwerveDebug")
        print("NetworkTables started: SwerveDebug should be available")


        self.driveMotor = SparkMax(driveMotorChannel, SparkMax.MotorType.kBrushless)
        self.turningMotor = SparkMax(turningMotorChannel, SparkMax.MotorType.kBrushless)

        driveMotorConfig = SparkMaxConfig()
        turnMotorConfig = SparkMaxConfig()

        driveMotorConfig.encoder.positionConversionFactor(ModuleConstants.kdriveFactor)
        driveMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kdriveFactor / 60.0)  # check this value

        turnMotorConfig.encoder.positionConversionFactor(ModuleConstants.kturnFactor)
        turnMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kturnFactor / 60.0)  # check this value

        self.driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        self.turningMotor.configure(turnMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        # Setup encoders for the drive and turning motors.
        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getEncoder()

        # Absolute Encoder
        self.absEncoder = AnalogEncoder(absEncoderChannel)
        self.absoluteEncoderOffsetRad = absouteEncoderOffset

        # Setup PID controllers for the driving and turning SPARKS MAX.
        self.drivePIDController = PIDController(ModuleConstants.kDriveP, ModuleConstants.kDriveI, ModuleConstants.kDriveD)
        self.turningPIDController = PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.KDTurning)
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

        self.driveFeedforward = SimpleMotorFeedforwardMeters(ModuleConstants.kDrivekS, ModuleConstants.kDrivekV)
        self.turnFeedforward = SimpleMotorFeedforwardRadians(ModuleConstants.kTurningkS, ModuleConstants.kTurningkV)

        # Set the idle mode for the motors to brake.
        driveMotorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        turnMotorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)

        self.resetEncoders()

    def getAbsoluteEncoderRad(self) -> float:
        """Returns the absolute encoder position in radians minus the offset."""
        raw = self.absEncoder.get()

        full_scale = 1.0172 #average full-scale value from testing
        angle = (raw / full_scale) * math.tau #convert normalized value to radians
        angle -= self.absoluteEncoderOffsetRad
        return angle

    def getDrivePosition(self) -> float:
        """Returns the position of the drive encoder in meters."""
        return self.driveEncoder.getPosition()

    def getTurningPosition(self) -> float:
        """Returns the position of the turning encoder in radians."""
        return self.turningEncoder.getPosition()

    def getDriveVelocity(self) -> float:
        """Returns the velocity of the drive encoder in meters per second."""
        return self.driveEncoder.getVelocity()

    def getTurningVelocity(self) -> float:
        """Returns the velocity of the turning encoder in radians per second."""
        return self.turningEncoder.getVelocity()

    def resetEncoders(self):
        """Resets the drive encoder to zero and turning encoder to the absolute encoder value."""
        self.driveEncoder.setPosition(0)
        self.turningEncoder.setPosition(self.getAbsoluteEncoderRad())

    def getSwerveModulePosition(self) -> SwerveModulePosition:
        """Returns the current position of the module."""
        return SwerveModulePosition(self.getDrivePosition(), Rotation2d(self.getAbsoluteEncoderRad()))

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module."""
        return SwerveModuleState(self.getDriveVelocity(), Rotation2d(self.getTurningPosition()))

    def getModuleStates(self):
        """Convenience method to return the state of this module (for use in drivetrain code)."""
        return self.getState()
    
    
    def optimizeState(self, desiredState: SwerveModuleState, currentAngle: Rotation2d) -> SwerveModuleState:
        delta = desiredState.angle - currentAngle
        if abs(delta.radians()) > math.pi / 2:
            return SwerveModuleState(-desiredState.speed, desiredState.angle + Rotation2d(math.pi))
        return desiredState


    def setDesiredState(self, state: SwerveModuleState):
        """Sets the desired state of the module."""
        self.state = self.optimizeState(state, self.getState().angle)

        try:
            self.driveMotor.set(self.state.speed / ModuleConstants.kmaxModuleVelocity)
        except Exception as e:
            print(f"Drive motor set error: {e}")
            self.driveMotor.set(0)

        try:
            self.turningMotor.set(self.turningPIDController.calculate(self.getAbsoluteEncoderRad(), self.state.angle.radians()))
        except Exception as e:
            print(f"Turning motor set error: {e}")
            self.turningMotor.set(0)

    def stop(self):
        """Stops the module."""
        self.driveMotor.set(0)
        self.turningMotor.set(0)
