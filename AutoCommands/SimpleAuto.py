
import time
import commands2
from subsystems.drivesubsystem import DriveSubsystem
from commands2 import Command


from subsystems.drivesubsystem import DriveSubsystem

class SimpleAuto(Command):
    """A simple autonomous routine for testing robot movement in AdvantageScope."""

    def __init__(self, robotDrive: DriveSubsystem):
        super().__init__()
        self.robotDrive = robotDrive
        self.start_time = 0

        #  Requires the drive subsystem
        self.addRequirements(self.robotDrive)

    def initialize(self):
        """Called once when autonomous starts."""
        self.start_time = time.time()
        print(" Simple Autonomous Started")

    def execute(self):
        """Runs periodically during autonomous mode and logs swerve states."""
        elapsed_time = time.time() - self.start_time

        if elapsed_time < 2.0:
            # Move forward with more speed to verify movement
            self.robotDrive.drive(1.0, 0.0, 0.0, fieldRelative=False, rateLimit=False)
        elif elapsed_time < 4.0:
            # Rotate to see if robot turns in AS
            self.robotDrive.drive(0.0, 0.0, 1.0, fieldRelative=False, rateLimit=False)
        else:
            # Stop after 4 seconds
            self.robotDrive.drive(0, 0, 0, False, False)

        #  Print swerve module states to see if they are updating
        states = self.robotDrive.getModuleStates()
        print(f"Auto Running - Time: {elapsed_time:.2f} sec, States: {states}")

        #  Log the values to NetworkTables
        self.robotDrive.publishSwerveStates()


    def isFinished(self):
        """End the command after 4 seconds."""
        return time.time() - self.start_time >= 4.0

    def end(self, interrupted):
        """Stop the robot when the command ends."""
        self.robotDrive.drive(0, 0, 0, False, False)
        print(" Simple Autonomous Ended")
