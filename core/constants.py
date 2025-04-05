from wpimath import units
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Translation2d
from wpimath.kinematics import MecanumDriveKinematics as Kinematics
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from navx import AHRS
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPLTVController
from pathplannerlib.pathfinding import PathConstraints
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib import logger, utils
from lib.classes import Alliance, PID, Tolerance, MotorControllerType, DifferentialModuleConstants, DifferentialModuleConfig, DifferentialModuleLocation, PoseSensorConfig, DriftCorrectionConstants, TargetAlignmentConstants
from core.classes import Target, TargetType, TargetAlignmentLocation
import math


APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout().loadField(AprilTagField.k2025ReefscapeAndyMark)
PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    kTrackWidth: units.meters = units.inchesToMeters(23)
    kWheelBase: units.meters = units.inchesToMeters(27.5)

    kTranslationSpeedMax: units.meters_per_second = 4.8
    kRotationSpeedMax: units.radians_per_second = 2 * math.pi # type: ignore

    kInputLimitDemo: units.percent = 0.5
    kInputRateLimitDemo: units.percent = 0.33

    _differentialModuleConstants = DifferentialModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      drivingMotorControllerType = MotorControllerType.SparkMax,
      drivingMotorCurrentLimit = 50,
      drivingMotorReduction = 8.46
    )

    kDifferentialModuleConfigs: tuple[DifferentialModuleConfig, ...] = (
      DifferentialModuleConfig(DifferentialModuleLocation.LeftFront, 10, None, True, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.LeftRear, 11, None, True, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.RightFront, 12, None, False, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.RightRear, 13, None, False, _differentialModuleConstants)
    )

    kDriveKinematics = Kinematics(
      Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
      Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
      Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
      Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
    )

    kPathPlannerRobotConfig = PATHPLANNER_ROBOT_CONFIG
    kPathPlannerController = PPLTVController(0.02)
    kPathFindingConstraints = PathConstraints(2.4, 1.6, units.degreesToRadians(540), units.degreesToRadians(720))

    kDriftCorrectionConstants = DriftCorrectionConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationTolerance = Tolerance(0.5, 1.0)
    )

    kTargetAlignmentConstants = TargetAlignmentConstants(
      rotationPID = PID(0.075, 0, 0.001),
      rotationTolerance = Tolerance(1.0, 2.0),
      rotationSpeedMax = kRotationSpeedMax * 0.5, 
      rotationHeadingModeOffset = 0.0,
      rotationTranslationModeOffset = 180,
      translationPID = PID(5.0, 0, 0),
      translationTolerance = Tolerance(0.05, 0.1),
      translationSpeedMax = kTranslationSpeedMax * 0.5
    )

  class Roller:
    kMotorCanId: int = 15
    kEjectSpeed: float = 1.0
    kReverseSpeed: float = -1.0

  class AlgaeRemover:
    kMotorCanId: int = 16
    kRotationSpeed: float = 0.5
    kReverseRotationSpeed: float = -0.5

  class Climber:
    kMotorCanId: int = 14
    kClimbSpeed: float = 1.0
    kReverseSpeed: float = -1.0

class Services:
  class Localization:
    kStateStandardDeviations: tuple[float, float, float] = (0.05, 0.05, units.degreesToRadians(5))
    kVisionMultiTagStandardDeviations: tuple[float, float, float] = (0.1, 0.1, units.degreesToRadians(10))
    kVisionDefaultStandardDeviations: tuple[float, float, float] = (0.5, 0.5, units.degreesToRadians(15))
    kVisionMaxPoseAmbiguity: units.percent = 0.2

class Sensors:
  class Gyro:
    class NAVX2:
      kComType = AHRS.NavXComType.kUSB1

  class Pose:
    _poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    _fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    
    kPoseSensorConfigs: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        "Front",
        Transform3d(
          Translation3d(units.inchesToMeters(0), units.inchesToMeters(0), units.inchesToMeters(0)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(0), units.degreesToRadians(0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      ),
    )

  class Camera:
    kStreams: dict[str, str] = {
      "Front": "http://10.27.89.6:1184/?action=stream",
      "Driver": "http://10.27.89.6:1188/?action=stream"
    }

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: units.percent = 0.1

class Game:
  class Commands:
    kAutoMoveTimeout: units.seconds = 4.0

  class Field:
    kAprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT
    kLength = APRIL_TAG_FIELD_LAYOUT.getFieldLength()
    kWidth = APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
    kBounds = (Translation2d(0, 0), Translation2d(kLength, kWidth))

    class Targets:
      kTargets: dict[Alliance, dict[int, Target]] = {
        Alliance.Red: {
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(1).toPose2d()): Target(TargetType.Station, APRIL_TAG_FIELD_LAYOUT.getTagPose(1)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(2).toPose2d()): Target(TargetType.Station, APRIL_TAG_FIELD_LAYOUT.getTagPose(2)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(3).toPose2d()): Target(TargetType.Processor, APRIL_TAG_FIELD_LAYOUT.getTagPose(3)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(4).toPose2d()): Target(TargetType.Barge, APRIL_TAG_FIELD_LAYOUT.getTagPose(4)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(5).toPose2d()): Target(TargetType.Barge, APRIL_TAG_FIELD_LAYOUT.getTagPose(5)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(6).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(6)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(7).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(7)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(8).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(8)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(9).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(9)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(10).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(10)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(11).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(11))
        },
        Alliance.Blue: {
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(12).toPose2d()): Target(TargetType.Station, APRIL_TAG_FIELD_LAYOUT.getTagPose(12)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(13).toPose2d()): Target(TargetType.Station, APRIL_TAG_FIELD_LAYOUT.getTagPose(13)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(14).toPose2d()): Target(TargetType.Barge, APRIL_TAG_FIELD_LAYOUT.getTagPose(14)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(15).toPose2d()): Target(TargetType.Barge, APRIL_TAG_FIELD_LAYOUT.getTagPose(15)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(16).toPose2d()): Target(TargetType.Processor, APRIL_TAG_FIELD_LAYOUT.getTagPose(16)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(17).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(17)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(18).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(18)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(19).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(19)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(20).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(20)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(21).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(21)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(22).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(22))
        }
      }

      kTargetAlignmentTransforms: dict[TargetType, dict[TargetAlignmentLocation, Transform3d]] = {
        TargetType.Reef: {
          TargetAlignmentLocation.Default: Transform3d(),
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(24), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(24), units.inchesToMeters(-6.5), 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(24), units.inchesToMeters(6.5), 0, Rotation3d())
        },
        TargetType.Station: {
          TargetAlignmentLocation.Default: Transform3d(),
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(0), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(0), units.inchesToMeters(-30), 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(0), units.inchesToMeters(30), 0, Rotation3d())
        },
        TargetType.Processor: {
          TargetAlignmentLocation.Default: Transform3d(),
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(24), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(24), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(24), 0, 0, Rotation3d())
        },
        TargetType.Barge: {
          TargetAlignmentLocation.Default: Transform3d(),
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d())
        }
      }
