from typing import Callable
from wpimath.geometry import Rotation2d, Pose2d, Pose3d, Transform3d
from wpimath.estimator import MecanumDrivePoseEstimator
from wpimath.kinematics import MecanumDriveWheelPositions
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib.sensors.pose_sensor import PoseSensor
from lib import utils
from core.classes import Target, TargetAlignmentLocation, TargetType
from ntcore import NetworkTableInstance


import core.constants as constants
import photonlibpy


class LocalizationService():
    def __init__(
      self,
      getGyroRotation: Callable[[], Rotation2d],
      getModulePositions: Callable[[], MecanumDriveWheelPositions],
      poseSensors: tuple[PoseSensor, ...]
    ) -> None:
        super().__init__()
        self._poseSensors = poseSensors
        self._getGyroRotation = getGyroRotation
        self._getModulePositions = getModulePositions

        self._poseEstimator = MecanumDrivePoseEstimator(
            kinematics=constants.Subsystems.Drive.kDriveKinematics,
            gyroAngle=self._getGyroRotation(),
            wheelPositions=self._getModulePositions(),
            initialPose=Pose2d()
            #constants.Services.Localization.kStateStandardDeviations,
            #constants.Services.Localization.kVisionDefaultStandardDeviations
        )

        self._photonCamera = photonlibpy.PhotonCamera("Arducam_OV9281_USB_Camera")
        self._photon_estimator = photonlibpy.PhotonPoseEstimator(
            constants.APRIL_TAG_FIELD_LAYOUT, 
            constants.Sensors.Pose._poseStrategy,
            self._photonCamera,
            constants.Sensors.Pose.kPoseSensorConfigs[0].cameraTransform
        )

        nt = NetworkTableInstance.getDefault()
        self._photon_estimation = nt.getStructTopic("Vision/EstimatedPose", Pose3d).publish()
        self._photon_estimation_2d = nt.getStructTopic("Vision/EstimatedPose2d", Pose2d).publish()
        self._gyro_readings = nt.getFloatTopic("Robot/Gyroscope").publish()

        self._photon_estimation.set(Pose3d())
        self._photon_estimation_2d.set(Pose2d())

        self.estimated_pose_3d = Pose3d()

        self._alliance = None
        self._robotPose = Pose2d()
        self._targets: dict[int, Target] = {}
        self._targetPoses: list[Pose2d] = []
    
    def _periodic(self) -> None:
        self._updateRobotPose()
        #self._updateTargets()
        self._updateTelemetry()

    def _updateRobotPose(self):
        # Arrow Pointing Down if Copy-Paste Wanted:  ↓
        # This should invert the gyro angle    ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓     if nessesary, no clue if it works though
        self._poseEstimator.update(gyroAngle= (self._getGyroRotation()), wheelPositions=self._getModulePositions())

        estimated_pose = self._photon_estimator.update()
        if estimated_pose:
            self.estimated_pose_3d = estimated_pose.estimatedPose
            estimated_pose_2d = estimated_pose.estimatedPose.toPose2d()
            self._poseEstimator.addVisionMeasurement(estimated_pose_2d, estimated_pose.timestampSeconds)

        self._robotPose = self._poseEstimator.getEstimatedPosition()

    def _updateTelemetry(self):
        self._photon_estimation.set(self.estimated_pose_3d)
        self._photon_estimation_2d.set(self._robotPose)
        self._gyro_readings.set(self._getGyroRotation().degrees())

    def getRobotPose(self) -> Pose2d:
        return self._robotPose

    def resetRobotPose(self, pose: Pose2d) -> None:
        self._poseEstimator.resetPose(pose)

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