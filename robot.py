#!/usr/bin/env python3
'''

This is the main class for the robot. This class is the glue that binds the robot together. All of the robot-wide systems
(such as the Field2d, RobotContainer, and RobotContainer) should be instantiated here. The autonomous routine should
be in the autonomousInit() method, and the teleop control should be in the teleopPeriodic() method. The robotInit() and
robotPeriodic() methods at the bottom of this file should generally remain unchanged. 

'''


import commands2
from commands2 import CommandScheduler
import wpilib
import typing
from robotcontainer import RobotContainer
from subsystems.swervemodule import SwerveModule


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        # Instantiate our RobotContainer.  
        self.container = RobotContainer()
        #self.autonomousCommand = None

        self.driverController = self.container.driverController

        #self.drive = self.Container.drive
        self.robotDrive = self.container.robotDrive
        #CommandScheduler.getInstance().registerSubsystem(self.robotDrive)

        global robotDrive #expose the drive subsystem globally (nothing can go wrong, right?)

        robotDrive = self.robotDrive


        wpilib.DataLogManager.start()  # Start logging
        wpilib.DriverStation.startDataLog(wpilib.DataLogManager.getLog())  # Log joystick inputs
        print("WPILib Data Logging Enabled")


    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        pass
        #print("Starting Autonomous... ")
        #self.autonomousCommand = self.container.getAutonomousCommand()
        #if self.autonomousCommand is not None:
        #    self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass
        #print("Starting TeleOp... ")
        #SwerveModule.resetEncoders()

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        CommandScheduler.getInstance().run()

    def testPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
