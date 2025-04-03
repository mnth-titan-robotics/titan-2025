from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
import wpilib
from wpimath.geometry import Transform2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from lib import logger, utils
from lib.classes import Alliance, TargetAlignmentMode
from lib.sensors.gyro_sensor_adis16470 import GyroSensor_ADIS16470
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import TargetAlignmentLocation
import core.constants as constants
from core.subsystems.drive import DriveSubsystem
from core.subsystems.roller import RollerSubsystem
from wpilib import ADIS16470_IMU as IMU

class AutoPath(Enum):
  Move1 = auto()

class AutoCommands:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

    self._paths = { path: PathPlannerPath.fromPathFile(path.name) for path in AutoPath }

    # AutoBuilder.configure(
    #   self._robot.localizationService.getRobotPose, 
    #   self._robot.localizationService.resetRobotPose, 
    #   self._robot.driveSubsystem.getChassisSpeeds, 
    #   self._robot.driveSubsystem.drive, 
    #   constants.Subsystems.Drive.kPathPlannerController,
    #   constants.Subsystems.Drive.kPathPlannerRobotConfig,
    #   lambda: utils.getAlliance() == Alliance.Red,
    #   self._robot.driveSubsystem
    # )

    self._autoCommandChooser = SendableChooser()
    self._autoCommandChooser.setDefaultOption("None", cmd.none)
    self._autoCommandChooser.addOption("Straight Forward", self.auto_straight)
    self._autoCommandChooser.addOption("Straight Then Right", self.auto_right_turn)
    self._autoCommandChooser.addOption("Straight Then Left", self.auto_left_turn)

    SmartDashboard.putData("Robot/Auto/Command", self._autoCommandChooser)

    # Drive Command Variables

    self.driveSubsystem = self._robot.driveSubsystem
    self.rollerSubsystem = self._robot.rollerSubsystem

    self.motor_speed = 0.25
    self.motor_stop_time = 0.1

    self.mode_0_drive_time = 3.25

    self.other_mode_step_1_drive_time = 1.85
    self.other_mode_step_2_drive_time = 1.75

    self.robot_turn_angle = 45.0

  def getSelected(self) -> Command:
    return self._autoCommandChooser.getSelected()()
  
  def _reset(self, path: AutoPath) -> Command:
    return cmd.sequence(
      AutoBuilder.resetOdom(self._paths.get(path).getPathPoses()[0].transformBy(Transform2d(0, 0, self._paths.get(path).getInitialHeading()))),
      cmd.waitSeconds(0.1)
    )
  
  def _move(self, path: AutoPath) -> Command:
    return cmd.sequence(
      AutoBuilder.followPath(self._paths.get(path))
    ).withTimeout(
      constants.Game.Commands.kAutoMoveTimeout
    )
  
  def _alignToTarget(self) -> Command:
    return cmd.sequence(self._robot.gameCommands.alignRobotToTargetCommand(TargetAlignmentMode.Translation, TargetAlignmentLocation.Left))
  
  def auto_straight(self) -> Command:
    return cmd.sequence(
      self.driveSubsystem.driveCommand(
        lambda: -(self.motor_speed),
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(self.mode_0_drive_time),
      self.driveSubsystem.driveCommand(
        lambda: 0.0,
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(self.motor_stop_time),
      self.rollerSubsystem.auto_ejectCommand(1)
    ).withName("AutoCommands:auto_straight")
  
  def auto_right_turn(self) -> Command:
    return cmd.sequence(
      self.driveSubsystem.driveCommand( # Drive forward beside reef
        lambda: -(self.motor_speed),
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(self.other_mode_step_1_drive_time),
      self.driveSubsystem.driveCommand( # Stop driving
        lambda: 0.0,
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(self.motor_stop_time),
      self.driveSubsystem.driveCommand( # Turn to face reef
        lambda: 0.0,
        lambda: 0.0,
        lambda: (self.robot_turn_angle)
      ).withTimeout(0.2),
      self.driveSubsystem.driveCommand( # Stop turning
        lambda: 0.0,
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(self.motor_stop_time),
      self.driveSubsystem.driveCommand( # Drive forward to reef
        lambda: -(self.motor_speed),
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(self.other_mode_step_2_drive_time),
      self.driveSubsystem.driveCommand( # Stop driving
        lambda: 0.0,
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(self.motor_stop_time),
      self.rollerSubsystem.auto_ejectCommand(1)
    ).withName("AutoCommands:auto_right_turn")
  
  def auto_left_turn(self) -> Command:
    return cmd.sequence(
      self.driveSubsystem.driveCommand( # Drive forward beside reef
        lambda: -(self.motor_speed),
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(self.other_mode_step_1_drive_time),
      self.driveSubsystem.driveCommand( # Stop driving
        lambda: 0.0,
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(self.motor_stop_time),
      self.driveSubsystem.driveCommand( # Turn to face reef
        lambda: 0.0,
        lambda: 0.0,
        lambda: -(self.robot_turn_angle)
      ).withTimeout(0.2),
      self.driveSubsystem.driveCommand( # Stop turning
        lambda: 0.0,
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(self.motor_stop_time),
      self.driveSubsystem.driveCommand( # Drive forward to reef
        lambda: -(self.motor_speed),
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(self.other_mode_step_2_drive_time),
      self.driveSubsystem.driveCommand( # Stop driving
        lambda: 0.0,
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(self.motor_stop_time),
      self.rollerSubsystem.auto_ejectCommand(1)
    ).withName("AutoCommands:auto_left_turn")
  