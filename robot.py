#!/usr/bin/env python3

import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import drivetrain
from drivetrain import kMaxSpeed, kMaxAngularSpeed, Drivetrain

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function - JL"""
        self.controller = wpilib.XboxController(0)
        self.swerve = drivetrain.Drivetrain()

        # # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1. - JL
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        # Reset encoders at the start of the match - JL
        self.swerve.resetEncoders()
        # Update odometry with the starting positions - JL
        self.swerve.updateOdometry()
        self.log_counter = 0
        self.log_interval = 50  # Log every 50 iterations - LC 1/30/25

    def autonomousPeriodic(self) -> None:
        #self.swerve.autonomousRoutine()
        self.logHeading()

    def teleopPeriodic(self) -> None:
        self.driveWithJoystick()
        self.logHeading()
        if self.log_counter % 50 == 0:
            self.swerve.showInverted()
        self.log_counter += 1

    def driveWithJoystick(self) -> None:
        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftY(), 0.1)
            )
            * kMaxSpeed
        )

        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftX(), 0.1)
            )
            * kMaxSpeed
        )

        rot = 0
        '''rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRightX(), 0.1)
            )
            * kMaxAngularSpeed
        )'''

        self.swerve.drive(xSpeed, ySpeed, rot, True, self.getPeriod())

    def logHeading(self) -> None:
        """Logs the robot's heading to the SmartDashboard. - JL"""
        heading = self.swerve.gyro.getAngle()
        wpilib.SmartDashboard.putNumber("Robot Heading", heading)

if __name__ == "__main__":
    wpilib.run(MyRobot)
