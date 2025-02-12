# Type: Swerve Module
''' 
This file contains the SwerveModule class, which represents a single swerve module on the robot.

'''




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

from wpimath.controller import PIDController, ProfiledPIDController,SimpleMotorFeedforwardMeters, SimpleMotorFeedforwardRadians

from constants import ModuleConstants
from rev import SparkMax, SparkMaxConfig, SparkBaseConfig, SparkBase
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from ntcore import NetworkTableInstance



class SwerveModule:
    def __init__(self, driveMotorChannel: int, turningMotorChannel: int, absEncoderChannel: int, absouteEncoderOffset: float) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder, and turning encoder. - JL

        :param driveMotorChannel: PWM output for the drive motor.
        :param turningMotorChannel: PWM output for the turning motor.
        :param turningEncoderChannel: Analog input for the turning absolute encoder.
        :param absoluteEncoderOffset: Offset for the absolute encoder.
        """


        # Force NetworkTables to start
        self.nt = NetworkTableInstance.getDefault()
        self.nt.startServer()  # Explicitly start the NT server

        # Create a NetworkTables entry for debugging
        self.debugTable = self.nt.getTable("SwerveDebug")

        # Log NT startup
        print("NetworkTables started: SwerveDebug should be available")


        self.absouteEncoderOffset = 0
        self.desiredState = SwerveModuleState(0.0, Rotation2d())



        self.driveMotor = SparkMax(driveMotorChannel, SparkMax.MotorType.kBrushless)
        self.turningMotor = SparkMax(turningMotorChannel, SparkMax.MotorType.kBrushless)



        #self.driveMotor.setInverted(False)
        #self.turningMotor.setInverted(False)

        driveMotorConfig = SparkMaxConfig()
        turnMotorConfig = SparkMaxConfig()


        driveMotorConfig.encoder.positionConversionFactor(ModuleConstants.kdriveFactor)
        driveMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kdriveFactor / 60.0) #! check this value

        turnMotorConfig.encoder.positionConversionFactor(ModuleConstants.kturnFactor)
        turnMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kturnFactor / 60.0) #! check this value

        self.driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        self.turningMotor.configure(turnMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)


        # Setup encoders for the drive and turning motors.
        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getEncoder()


        # Absolute Encoder
        # Absolute Encoders help "remember" the location of the module.
        # This is useful for when the robot is turned off and on again.
        self.absEncoder = AnalogEncoder(absEncoderChannel)
        self.absoluteEncoderOffsetRad = absouteEncoderOffset



        # Setup PID controllers for the driving and turning SPARKS MAX.
        self.drivePIDController = PIDController(ModuleConstants.kDriveP, ModuleConstants.kDriveI, ModuleConstants.kDriveD)
        self.turningPIDController = PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.KDTurning)
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

        self.driveFeedforward = SimpleMotorFeedforwardMeters(ModuleConstants.kDrivekS, ModuleConstants.kDrivekV)
        self.turnFeedforward = SimpleMotorFeedforwardRadians(ModuleConstants.kTurningkS, ModuleConstants.kTurningkV)   #! check if FF radians is correct

      
        # Set the idle mode for the motors to brake.
        driveMotorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        turnMotorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)



        self.chassisAngularOffset = absouteEncoderOffset
        self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition())
        self.driveEncoder.setPosition(0)

    
    def getAbsoluteEncoderRad(self) -> float:
        """Returns the absolute encoder position in radians minus the offset."""
        angle = self.absEncoder.get()
        angle *= math.tau #? convert to radians
        angle -= self.absoluteEncoderOffsetRad #? get acual location depending on the offset
        # TODO: calculate the actual offset, convert to rad, add here -JL on 2/3/25      
        return angle 

    def resetEncoders(self):
        """Resets the drive encoders to zero and Turn Encoders to value from absolute encoder in rads."""
        self.driveEncoder.setPosition(0)
        self.turningEncoder.setPosition(self.getAbsoluteEncoderRad())


    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module. - JL"""
        speed = self.driveEncoder.getVelocity()  #getVelocity here provides real-time speed of wheeel which is what we need not total distance traveled. - LC 2/4/25
        angle = wpimath.geometry.Rotation2d(    #! getVelocity returns the RPM of the motor -JL on 2/5/25
            self.getAbsoluteEncoderRad()
        )
        return wpimath.kinematics.SwerveModuleState(speed, angle)

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module."""
        position = self.driveEncoder.getPosition() * ModuleConstants.kPositionConversionFactor
        angle = wpimath.geometry.Rotation2d(
            self.getAbsoluteEncoderRad()
        )
        return wpimath.kinematics.SwerveModulePosition(position, angle)

    def setDesiredState(self, desiredState: wpimath.kinematics.SwerveModuleState) -> None:
        """Sets the desired state for the swerve module.
        
        :param desiredState: Desired state with speed and angle.
        """

        # correctedDesiredState = SwerveModuleState()
        # correctedDesiredState.speed = desiredState.speed
        # correctedDesiredState.angle = desiredState.angle + Rotation2d(self.chassisAngularOffset)

        correctedDesiredState = SwerveModuleState(
            desiredState.speed, 
            desiredState.angle + Rotation2d(self.chassisAngularOffset)
        )
        # Send corrected state to NetworkTables
        self.debugTable.putNumber("CorrectedSpeed", correctedDesiredState.speed)
        self.debugTable.putNumber("CorrectedAngle", correctedDesiredState.angle.radians())


                # Optimize the reference state to avoid spinning further than 90 degrees.
        optimizedDesiredState = SwerveModuleState.optimize(
            correctedDesiredState, 
            self.getState().angle)
        # Optimize the reference state to avoid spinning further than 90 degrees.
        #!~ self.getState().angle already accounts for the offset since angle comes from def getAbsoluteEncoderRad()
        #desiredState = wpimath.kinematics.SwerveModuleState.optimize(desiredState, Rotation2d(self.turningEncoder.getPosition()))
            # If optimization fails, log it
        if optimizedDesiredState is None:
            self.debugTable.putString("OptimizeError", "optimize() returned None!")
            optimizedDesiredState = correctedDesiredState  # Fallback
        
        # Send optimized state to NetworkTables
        self.debugTable.putNumber("OptimizedSpeed", optimizedDesiredState.speed)
        self.debugTable.putNumber("OptimizedAngle", optimizedDesiredState.angle.radians())

        # Send wheel vector data to NetworkTables for AdvantageScope visualization
        moduleName = f"Module_{self.driveMotor.getDeviceId()}"  # Unique name per module
        moduleTable = self.nt.getTable(f"SwerveModules/{moduleName}")

        moduleTable.putNumber("Speed", optimizedDesiredState.speed)
        moduleTable.putNumber("Angle", optimizedDesiredState.angle.radians())


        
    
            # Command driving and turning SPARKS MAX towards their respective setpoints.
        self.drivePIDController.setSetpoint(optimizedDesiredState.speed)
        self.turningPIDController.setSetpoint(optimizedDesiredState.angle.radians())

        # self.drivePIDController.setSetpoint(desiredState.speed, SparkMax.ControlType.kVelocity)

        # self.turningPIDController.setSetpoint(desiredState.angle.radians(), SparkMax.ControlType.kPosition)

        self.desiredState = desiredState
        
        
        # try:
        #     driveOutput = self.drivePIDController.calculate( #! error = optimizedState.speed - self.driveEncoder.getVelocity()
        #         self.driveEncoder.getVelocity(),               #! driveOutput = kP(0.01) * error
        #         desiredState.speed
        #     )
        #     driveFeedforward = self.driveFeedforward.calculate(desiredState.speed)
        #     self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        # except Exception as e:
        #     #print(f"Drive error: {e}")
        #     self.driveMotor.setVoltage(0)
    
        # # Turning motor output
        # try:
        #     turnOutput = self.turningPIDController.calculate(
        #         self.turningEncoder.getPosition() * ModuleConstants.kDistancePerRotation,
        #         desiredState.angle.radians()
        #     )
        #     turnFeedforward = self.turnFeedforward.calculate(
        #         self.turningPIDController.getSetpoint().velocity
        #     )
        #     self.turningMotor.setVoltage(turnOutput + turnFeedforward)
        # except Exception as e:
        #     #print(f"Turn error: {e}")
        #     self.turningMotor.setVoltage(0)





   
