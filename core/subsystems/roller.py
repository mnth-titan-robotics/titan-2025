from typing import Callable
import math
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
from ntcore import NetworkTableInstance

class RollerSubsystem(Subsystem):
  def __init__(self):
    self._motor = rev.SparkMax(constants.Subsystems.Roller.kMotorCanId, rev.SparkBase.MotorType.kBrushed)
    config = rev.SparkMaxConfig().inverted(True).smartCurrentLimit(20)
    self._motor.configure(config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)
    self._publisher = NetworkTableInstance.getDefault().getFloatTopic("Subsystem/Roller/Speed").publish()
    self._currentPublisher = NetworkTableInstance.getDefault().getFloatTopic("Subsystem/Roller/Current").publish()

  def periodic(self) -> None:
    self._updateTelemetry()
  
  def _updateTelemetry(self) -> None:
    self._publisher.set(self._motor.get())
    self._currentPublisher.set(self._motor.getOutputCurrent())


#original code before we
# def ejectCommand(self) -> Command:
    #return self.runOnce(lambda: self._motor.set(constants.Subsystems.Roller.kEjectSpeed)).finallyDo(lambda cancel: self._motor.set(0.0))

  def ejectCommand(self) -> Command:
    return self.run(
        lambda: self._motor.set(constants.Subsystems.Roller.kEjectSpeed)
      ).finallyDo(
        lambda cancel: self._motor.stopMotor()
      )
  
  def reverseCommand(self) -> Command:
    return self.run(
        lambda: self._motor.set(constants.Subsystems.Roller.kReverseSpeed)
      ).finallyDo(
        lambda cancel: self._motor.stopMotor()
      )