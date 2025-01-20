#!/usr/bin/env python3

import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import drivetrain
from drivetrain import kMaxSpeed, kMaxAngularSpeed

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.XboxController(0)
        self.swerve = drivetrain.Drivetrain()

        # # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        # self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        # self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        # self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        # Reset encoders at the start of the match
        self.swerve.resetEncoders()
        # Update odometry with the starting positions
        self.swerve.updateOdometry()

    def autonomousPeriodic(self) -> None:
        self.swerve.autonomousRoutine()
        self.logHeading()

    def teleopPeriodic(self) -> None:
        self.driveWithJoystick()
        self.logHeading()

    def driveWithJoystick(self) -> None:
        xSpeed = (
            -wpimath.filter.SlewRateLimiter(3).calculate(
                wpimath.applyDeadband(self.controller.getLeftY(), 0.1)
            )
            * kMaxSpeed
        )

        ySpeed = (
            -wpimath.filter.SlewRateLimiter(3).calculate(
                wpimath.applyDeadband(self.controller.getLeftX(), 0.1)
            )
            * kMaxSpeed
        )

        rot = (
            -wpimath.filter.SlewRateLimiter(3).calculate(
                wpimath.applyDeadband(self.controller.getRightX(), 0.1)
            )
            * kMaxAngularSpeed
        )

        self.swerve.drive(xSpeed, ySpeed, rot, True, self.getPeriod())

    def logHeading(self) -> None:
        """Logs the robot's heading to the SmartDashboard."""
        heading = self.swerve.gyro.getAngle()
        wpilib.SmartDashboard.putNumber("Robot Heading", heading)

if __name__ == "__main__":
    wpilib.run(MyRobot)