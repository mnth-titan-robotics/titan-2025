from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard, DriverStation
from wpimath.geometry import Transform2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from lib.classes import TargetAlignmentMode
from core.classes import TargetAlignmentLocation
import math
import wpiutil

if TYPE_CHECKING: from core.robot import RobotCore
import core.constants as constants


class AutoPath(Enum):
  Move1 = auto()

class AutoCommands:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot
    self.camera = self._robot._photonCamera
    self.localization = self._robot.localizationService
    self.alliance = None

    self.aprilTagIsVisible = None
    self.aprilTagYaw = None

    self._paths = { path: PathPlannerPath.fromPathFile(path.name) for path in AutoPath }

    self._autoCommandChooser = SendableChooser()
    self._autoCommandChooser.setDefaultOption("None", cmd.none)
    self._autoCommandChooser.addOption("Straight Forward", self.auto_straight)
    self._autoCommandChooser.addOption("Straight Then Right", self.auto_right_turn)
    self._autoCommandChooser.addOption("Straight Then Left", self.auto_left_turn)

    #self._allianceSelectorChooser = SendableChooser()
    #self._allianceSelectorChooser.setDefaultOption("None", cmd.none)
    #self._allianceSelectorChooser.addOption("Blue Alliance", self.setBlueAlliance)
    #self._allianceSelectorChooser.addOption("Red Alliance", self.setRedAlliance)

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
    self.turnTime = 0.2
    self.isAtTarget = False

  def getSelected(self) -> Command:
    # This line will run before the autoCommandChooser runs a selected command from SmartDashboard
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
  
  def setBlueAlliance(self):
    self.alliance = "blue"

  def setRedAlliance(self):
    self.alliance = "red"
  
  def getCameraVariables(self, aprilTagID: list[int]):  
    if self.camera.isConnected():
      results = self.camera.getAllUnreadResults()
      if results and len(results):
        result = results[-1]
        # for result in results:
        for target in result.getTargets():
          if target.getFiducialId() in aprilTagID:
            return target.getYaw()

  def getAngleToAprilTag(self, aprilTagID: int) -> float:
    tagPose = constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(aprilTagID).toPose2d()
    robotPose = self.localization.getRobotPose()
    relativePose = tagPose.relativeTo(robotPose)
    translation = relativePose.translation()
    return -translation.angle().degrees()
  
  def turnToAprilTag(self, direction: str) -> float:
    # my brain is not big enough to code this -lead programmer
    self._autoCommandChooser.getSelected()()
    print(self.alliance)

    kMaxSpeed = 0.20
    direction = direction.lower()
    multFactor = 1
    self.isAtTarget = False

    intendedAprilTag = None
    intendedAprilTags = None

    # For turning left, we go to tag 11 for when we're on red alliance, 20 when we're on blue alliance
    # For turning right, we go to tag 9 for when we're on red alliance, 22 when we're on blue alliance
    # The intendedAprilTags variable is keyed = [red alliance tag, blue alliance tag]
    
    if direction == "left": 
      intendedAprilTags = [20, 11]
      multFactor = -1

    elif direction == "right":
      intendedAprilTags = [22, 9]
      multFactor = 1

    # Determine which tag we want based on our alliance

    if self.alliance == "red":
      intendedAprilTag = intendedAprilTags[1]

    elif self.alliance == "blue":
      intendedAprilTag = intendedAprilTags[0]

    else:
      print(f"Oh fuck the self.alliance variable was actually this: {self.alliance}")

    print(f"intedAprilTag: type={type(intendedAprilTag)} value={intendedAprilTag}")

    #SmartDashboard.putNumber("Vision/TargetYaw2", self.getAngleToAprilTag(intendedAprilTag))
    # The multFactor variable controls whether we're turning right or left
    # If multFactor is positive, then we are going to turn to the right
    # If multFactor is negative, then we are going to turn to the left

    targetYaw = self.getAngleToAprilTag(intendedAprilTag)
    # print("getting cam vars")
    # targetYaw = self.getCameraVariables(aprilTagID=intendedAprilTag)
    
    if not targetYaw:
        print("Turning to find apriltag")
        return multFactor * kMaxSpeed
      
    rotation = targetYaw / 15.0
    rotation = max(
      min(rotation, kMaxSpeed),
      -kMaxSpeed
    )
    #SmartDashboard.putNumber("Vision/TargetYaw", targetYaw)
    #SmartDashboard.putNumber("Vision/Rotation", rotation)
    
    if abs(targetYaw) < 2.0:
        print(rotation)
        rotation = 0.0
        self.isAtTarget = True
        return rotation

    return multFactor * 0.10
  
  def auto_straight(self) -> Command:
    return cmd.sequence(
      self.driveSubsystem.driveCommand(
        lambda: self.motor_speed,
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
        lambda: self.motor_speed,
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
        lambda: 45.0
      ).withTimeout(self.turnTime),
      # ).until(lambda: self.isAtTarget),
      self.driveSubsystem.driveCommand( # Stop turning
        lambda: 0.0,
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(self.motor_stop_time),
      self.driveSubsystem.driveCommand( # Drive forward to reef
        lambda: self.motor_speed,
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
        lambda: self.motor_speed,
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
        lambda: -45.0
      ).withTimeout(self.turnTime),
      # ).until(lambda: self.isAtTarget),
      self.driveSubsystem.driveCommand( # Stop turning
        lambda: 0.0,
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(self.motor_stop_time),
      self.driveSubsystem.driveCommand( # Drive forward to reef
        lambda: self.motor_speed,
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
  