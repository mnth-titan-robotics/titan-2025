from typing import Callable
from wpimath.geometry import Rotation2d, Pose2d, Pose3d
from wpimath.estimator import DifferentialDrivePoseEstimator, MecanumDrivePoseEstimator
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib.sensors.pose_sensor import PoseSensor
from lib.classes import DifferentialDriveModulePositions
from lib import utils
from core.classes import Target, TargetAlignmentLocation, TargetType
import core.constants as constants

class LocalizationService():
  def __init__(
      self,
      getGyroRotation: Callable[[], Rotation2d],
      getModulePositions: Callable[[], DifferentialDriveModulePositions],
      poseSensors: tuple[PoseSensor, ...]
    ) -> None:
    super().__init__()
    self._poseSensors = poseSensors
    self._getGyroRotation = getGyroRotation
    self._getModulePositions = getModulePositions

    self._poseEstimator = DifferentialDrivePoseEstimator(
      constants.Subsystems.Drive.kDriveKinematics,
      self._getGyroRotation(),
      *self._getModulePositions(),
      Pose2d(),
      constants.Services.Localization.kStateStandardDeviations,
      constants.Services.Localization.kVisionDefaultStandardDeviations
    )

    self._alliance = None
    self._robotPose = Pose2d()
    self._targets: dict[int, Target] = {}
    self._targetPoses: list[Pose2d] = []

  def _periodic(self) -> None:
    self._updateRobotPose()
    self._updateTargets()
    self._updateTelemetry()

  def _updateRobotPose(self) -> None:
    self._poseEstimator.update(self._getGyroRotation(), *self._getModulePositions())
    for poseSensor in self._poseSensors:
      estimatedRobotPose = poseSensor.getEstimatedRobotPose()
      if estimatedRobotPose is not None:
        pose = estimatedRobotPose.estimatedPose.toPose2d()
        if utils.isPoseInBounds(pose, constants.Game.Field.kBounds):
          if estimatedRobotPose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR:
            self._poseEstimator.addVisionMeasurement(pose, estimatedRobotPose.timestampSeconds, constants.Services.Localization.kVisionMultiTagStandardDeviations)
          else:
            ambiguity = sum(target.getPoseAmbiguity() for target in estimatedRobotPose.targetsUsed) / len(estimatedRobotPose.targetsUsed)
            if utils.isValueInRange(ambiguity, 0, constants.Services.Localization.kVisionMaxPoseAmbiguity):
              self._poseEstimator.addVisionMeasurement(pose, estimatedRobotPose.timestampSeconds, constants.Services.Localization.kVisionDefaultStandardDeviations)
    self._robotPose = self._poseEstimator.getEstimatedPosition()

  def getRobotPose(self) -> Pose2d:
    return self._robotPose

  def resetRobotPose(self, pose: Pose2d) -> None:
    self._poseEstimator.resetPose(pose)

  def _updateTargets(self) -> None:
    if utils.getAlliance() != self._alliance:
      self._alliance = utils.getAlliance()
      self._targets = constants.Game.Field.Targets.kTargets[self._alliance]
      self._targetPoses = [t.pose.toPose2d() for t in self._targets.values()]

  def getTargetPose(self, targetAlignmentLocation: TargetAlignmentLocation, targetType: TargetType) -> Pose3d:
    match targetType:
      case _:
        target = self._targets.get(utils.getTargetHash(self._robotPose.nearest(self._targetPoses)))
        return target.pose.transformBy(constants.Game.Field.Targets.kTargetAlignmentTransforms[target.type][targetAlignmentLocation])
  
  def hasVisionTargets(self) -> bool:
    for poseSensor in self._poseSensors:
      if poseSensor.hasTarget():
        return True
    return False

  def _updateTelemetry(self) -> None:
    pass
    