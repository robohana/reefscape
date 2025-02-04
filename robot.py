
#!/usr/bin/env python3

import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import drivetrain
from drivetrain import kMaxSpeed, kMaxAngularSpeed, Drivetrain
from constants import RobotConstants, ModuleConstants, OIConstants, DrivingConstants


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function - JL"""
        self.controller = wpilib.XboxController(OIConstants.kDriverControllerPort)
        self.swerve = drivetrain.Drivetrain()

        # # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1. - JL
        self.xspeedLimiter = DrivingConstants.voltageLimiter
        self.yspeedLimiter = DrivingConstants.voltageLimiter
        self.rotLimiter = DrivingConstants.voltageLimiter

        # Reset encoders at the start of the match - JL
        #self.swerve.resetEncoders() # TODO: REMOVE? we have this in swervemodule already. - JL 2/4/25
        # Update odometry with the starting positions - JL
        self.swerve.updateOdometry()
        # self.log_counter = 0
        # self.log_interval = 50  # Log every 50 iterations - LC 1/30/25

    def autonomousPeriodic(self) -> None:
        #self.swerve.autonomousRoutine()
        self.logHeading()

    def teleopPeriodic(self) -> None:
        self.driveWithJoystick()
        self.logHeading()
        # if self.log_counter % 50 == 0:
        #     self.swerve.showInverted()
        # self.log_counter += 1

    def driveWithJoystick(self) -> None:
        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(1), OIConstants.deadzone)
            )
            * kMaxSpeed
        )

        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(0), OIConstants.deadzone)
            )
            * kMaxSpeed
        )

        rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(4), OIConstants.deadzone)
            )
            * kMaxAngularSpeed
        )

        self.swerve.drive(xSpeed, ySpeed, rot, True, self.getPeriod())
        wpilib.SmartDashboard.putNumber("xSpeed", xSpeed)
        wpilib.SmartDashboard.putNumber("ySpeed", ySpeed)
        wpilib.SmartDashboard.putNumber("rot", rot)


    def logHeading(self) -> None:
        """Logs the robot's heading to the SmartDashboard. - JL"""
        heading = self.swerve.gyro.getAngle()
        wpilib.SmartDashboard.putNumber("Robot Heading", heading)

if __name__ == "__main__":
    wpilib.run(MyRobot)
