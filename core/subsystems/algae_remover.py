from commands2 import Subsystem, Command
from wpilib import SmartDashboard, SendableChooser
from wpilib.drive import DifferentialDrive
from wpimath import units
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveWheelSpeeds
from lib import utils, logger
from lib.classes import DifferentialModuleLocation, DifferentialDriveModulePositions, MotorIdleMode, SpeedMode, DriveOrientation, OptionState, TargetAlignmentMode
from lib.components.differential_module import DifferentialModule
from core.classes import TargetAlignmentLocation, TargetType
import core.constants as constants
import rev
import time


class AlgaeRemoverSubsystem(Subsystem):
    def __init__(self):
        self._motor = rev.SparkMax(constants.Subsystems.AlgaeRemover.kMotorCanId, rev.SparkBase.MotorType.kBrushless)
        config = rev.SparkMaxConfig().inverted(True).smartCurrentLimit(20)
        
        self._motor.configure(config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

    def periodic(self) -> None:
        pass
  
    def _updateTelemetry(self) -> None:
        pass

    def extendCommand(self) -> Command:
        return self.run(
            lambda: self._motor.set(constants.Subsystems.AlgaeRemover.kRotationSpeed)
        ).finallyDo(
            lambda cancel: self._motor.stopMotor()
        )
    
    def auto_extendCommand(self, set_timeout) -> Command:
        return self.run(
            lambda: self._motor.set(constants.Subsystems.AlgaeRemover.kRotationSpeed)
        ).withTimeout(set_timeout
        ).finallyDo(
            lambda cancel: self._motor.stopMotor()
        )
    
    # Are you guys not expecting to be able to remove algae while driving
    # Can't use pygame anyway because 1. you don't have it installed and 2. it's stupid considering the other code we have
    
    def retractCommand(self) -> Command:
        return self.run(
            lambda: self._motor.set(constants.Subsystems.AlgaeRemover.kReverseRotationSpeed)
        ).finallyDo(
            lambda cancel: self._motor.stopMotor()
        )