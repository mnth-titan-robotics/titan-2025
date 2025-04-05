from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SendableChooser
from wpilib.drive import MecanumDrive
from wpimath import units
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveWheelSpeeds
from lib import utils
from lib.classes import DifferentialModuleLocation, DifferentialDriveModulePositions, MotorIdleMode, SpeedMode, DriveOrientation, OptionState, TargetAlignmentMode
from lib.components.differential_module import DifferentialModule
from core.classes import TargetAlignmentLocation, TargetType
import core.constants as constants

class DriveSubsystem(Subsystem):
  def __init__(
      self, 
      getGyroHeading: Callable[[], units.degrees]
    ) -> None:
    super().__init__()
    self._getGyroHeading = getGyroHeading
    
    self._constants = constants.Subsystems.Drive

    self._differentialModules = dict((c.location, DifferentialModule(c)) for c in self._constants.kDifferentialModuleConfigs)
    
    self._drivetrain = MecanumDrive(
      self._differentialModules[DifferentialModuleLocation.LeftFront].getMotorController(),
      self._differentialModules[DifferentialModuleLocation.LeftRear].getMotorController(),
      self._differentialModules[DifferentialModuleLocation.RightFront].getMotorController(),
      self._differentialModules[DifferentialModuleLocation.RightRear].getMotorController()
    )

    self._isDriftCorrectionActive: bool = False
    self._driftCorrectionController = PIDController(*self._constants.kDriftCorrectionConstants.rotationPID)
    self._driftCorrectionController.setTolerance(*self._constants.kDriftCorrectionConstants.rotationTolerance)
    self._driftCorrectionController.enableContinuousInput(-180.0, 180.0)

    self._isAlignedToTarget: bool = False
    self._targetAlignmentRotationController = PIDController(*self._constants.kTargetAlignmentConstants.rotationPID)
    self._targetAlignmentRotationController.setTolerance(*self._constants.kTargetAlignmentConstants.rotationTolerance)
    self._targetAlignmentRotationController.enableContinuousInput(-180.0, 180.0)
    self._targetAlignmentTranslationXController = PIDController(*self._constants.kTargetAlignmentConstants.translationPID)
    self._targetAlignmentTranslationXController.setTolerance(*self._constants.kTargetAlignmentConstants.translationTolerance)
    self._targetAlignmentTranslationXController.setSetpoint(0)
    self._targetAlignmentTranslationYController = PIDController(*self._constants.kTargetAlignmentConstants.translationPID)
    self._targetAlignmentTranslationYController.setTolerance(*self._constants.kTargetAlignmentConstants.translationTolerance)
    self._targetAlignmentTranslationYController.setSetpoint(0)
    self._targetPose: Pose3d = None

    self._inputXFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
    self._inputYFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
    self._inputRotationFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)

    self._speedMode: SpeedMode = SpeedMode.Competition
    speedModeChooser = SendableChooser()
    speedModeChooser.setDefaultOption(SpeedMode.Competition.name, SpeedMode.Competition)
    speedModeChooser.addOption(SpeedMode.Demo.name, SpeedMode.Demo)
    speedModeChooser.onChange(lambda speedMode: setattr(self, "_speedMode", speedMode))
    #SmartDashboard.putData("Robot/Drive/SpeedMode", speedModeChooser)

    self._orientation: DriveOrientation = DriveOrientation.Field
    orientationChooser = SendableChooser()
    orientationChooser.setDefaultOption(DriveOrientation.Field.name, DriveOrientation.Field)
    orientationChooser.addOption(DriveOrientation.Robot.name, DriveOrientation.Robot)
    orientationChooser.onChange(lambda orientation: setattr(self, "_orientation", orientation))
    #SmartDashboard.putData("Robot/Drive/Orientation", orientationChooser)

    self._driftCorrection: OptionState = OptionState.Enabled
    driftCorrectionChooser = SendableChooser()
    driftCorrectionChooser.setDefaultOption(OptionState.Enabled.name, OptionState.Enabled)
    driftCorrectionChooser.addOption(OptionState.Disabled.name, OptionState.Disabled)
    driftCorrectionChooser.onChange(lambda driftCorrection: setattr(self, "_driftCorrection", driftCorrection))
    #SmartDashboard.putData("Robot/Drive/DriftCorrection", driftCorrectionChooser)

    idleModeChooser = SendableChooser()
    idleModeChooser.setDefaultOption(MotorIdleMode.Brake.name, MotorIdleMode.Brake)
    idleModeChooser.addOption(MotorIdleMode.Coast.name, MotorIdleMode.Coast)
    idleModeChooser.onChange(lambda idleMode: self._setIdleMode(idleMode))
    #SmartDashboard.putData("Robot/Drive/IdleMode", idleModeChooser)

    #SmartDashboard.putNumber("Robot/Drive/Chassis/Length", self._constants.kWheelBase)
    #SmartDashboard.putNumber("Robot/Drive/Chassis/Width", self._constants.kTrackWidth)
    #SmartDashboard.putNumber("Robot/Drive/Speed/Max", self._constants.kTranslationSpeedMax)

  def periodic(self) -> None:
    self._updateTelemetry()

  def driveCommand(
      self, 
      getInputX: Callable[[], float], 
      getInputY: Callable[[], float], 
      getInputRotation: Callable[[], float]
    ) -> Command:
    return self.run(
      lambda: self._drive(getInputX(), getInputY(), getInputRotation())
    ).withName("DriveSubsystem:Drive")

  def _drive(self, inputX: float, inputY: float, inputRotation: float) -> None:
    #RateLimit = 0.5
    #SRL = SlewRateLimiter(rateLimit=RateLimit)

    #inputRotation_Limited = SRL.calculate(inputRotation)
    # Theoretically ^ this ^ should be input scaling
    # This causes issues with rotation where it just doesn't so idk

    self._drivetrain.driveCartesian(
      xSpeed=inputX,
      ySpeed=inputY,
      zRotation=inputRotation,
      gyroAngle=Rotation2d()  #.fromDegrees(self._getGyroHeading())

      #drift correction goes here
    )

  def drive(self, chassisSpeeds: ChassisSpeeds) -> None:
    wheelSpeeds = self._constants.kDriveKinematics.toWheelSpeeds(chassisSpeeds)
    # TODO: Fix this, or get rid of this method
    # self._drivetrain.tankDrive(wheelSpeeds.left, wheelSpeeds.right)
    self.clearTargetAlignment()

  def getModulePositions(self) -> DifferentialDriveModulePositions:
    return DifferentialDriveModulePositions(
      self._differentialModules[DifferentialModuleLocation.LeftRear].getPosition(),
      self._differentialModules[DifferentialModuleLocation.RightRear].getPosition()
    )

  def getChassisSpeeds(self) -> ChassisSpeeds:
    return self._constants.kDriveKinematics.toChassisSpeeds(
      DifferentialDriveWheelSpeeds(
        self._differentialModules[DifferentialModuleLocation.LeftRear].getVelocity(), 
        self._differentialModules[DifferentialModuleLocation.RightRear].getVelocity()
      )
    )

  def _setIdleMode(self, idleMode: MotorIdleMode) -> None:
    # TODO: implement idleMode change on motor controllers
    #SmartDashboard.putString("Robot/Drive/IdleMode/selected", idleMode.name)
    pass

  def alignToTargetCommand(
      self, 
      getRobotPose: Callable[[], Pose2d], 
      getTargetPose: Callable[[TargetAlignmentLocation], Pose3d], 
      targetAlignmentMode: TargetAlignmentMode, 
      targetAlignmentLocation: TargetAlignmentLocation,
      targetType: TargetType
    ) -> Command:
    return self.run(
      lambda: self._runTargetAlignment(getRobotPose(), targetAlignmentMode)
    ).beforeStarting(
      lambda: self._initTargetAlignment(getRobotPose(), getTargetPose(targetAlignmentLocation, targetType), targetAlignmentMode)
    ).until(
      lambda: self._isAlignedToTarget
    ).withName("DriveSubsystem:AlignToTarget")
  
  def _initTargetAlignment(
      self, 
      robotPose: Pose2d, 
      targetPose: Pose3d, 
      targetAlignmentMode: TargetAlignmentMode
    ) -> None:
    self.clearTargetAlignment()
    self._targetPose = targetPose
    self._targetAlignmentRotationController.reset()
    if targetAlignmentMode == TargetAlignmentMode.Heading:
      self._targetAlignmentRotationController.setSetpoint(utils.wrapAngle(utils.getTargetHeading(robotPose, targetPose) + self._constants.kTargetAlignmentConstants.rotationHeadingModeOffset))
    else:
      self._targetAlignmentRotationController.setSetpoint(targetPose.toPose2d().rotation().degrees() + self._constants.kTargetAlignmentConstants.rotationTranslationModeOffset)
    self._targetAlignmentTranslationXController.reset()
    self._targetAlignmentTranslationYController.reset()
    
  def _runTargetAlignment(self, robotPose: Pose2d, targetAlignmentMode: TargetAlignmentMode) -> None:
    targetTranslation = self._targetPose.__sub__(Pose3d(robotPose))

    speedRotation = 0
    speedTranslationX = 0
    speedTranslationY = 0

    # TODO: implement differential drive target alignment logic and drive values

    # if not self._targetAlignmentRotationController.atSetpoint():
    #   speedRotation = self._targetAlignmentRotationController.calculate(robotPose.rotation().degrees())

    # if targetAlignmentMode == TargetAlignmentMode.Translation and not self._targetAlignmentTranslationXController.atSetpoint():
    #   speedTranslationX = self._targetAlignmentTranslationXController.calculate(targetTranslation.X())

    # if targetAlignmentMode == TargetAlignmentMode.Translation and not self._targetAlignmentTranslationYController.atSetpoint():
    #   speedTranslationY = self._targetAlignmentTranslationYController.calculate(targetTranslation.Y())

    # self._setSwerveModuleStates(
    #   self._constants.kDriveKinematics.toSwerveModuleStates(
    #     ChassisSpeeds(
    #       -utils.clampValue(speedTranslationX, -self._constants.kTargetAlignmentConstants.translationSpeedMax, self._constants.kTargetAlignmentConstants.translationSpeedMax), 
    #       -utils.clampValue(speedTranslationY, -self._constants.kTargetAlignmentConstants.translationSpeedMax, self._constants.kTargetAlignmentConstants.translationSpeedMax),
    #       utils.clampValue(speedRotation, -self._constants.kTargetAlignmentConstants.rotationSpeedMax, self._constants.kTargetAlignmentConstants.rotationSpeedMax)
    #     )
    #   )
    # )

    if speedRotation == 0 and speedTranslationX == 0 and speedTranslationY == 0:
      self._isAlignedToTarget = True

  def isAlignedToTarget(self) -> bool:
    return self._isAlignedToTarget
  
  def clearTargetAlignment(self) -> None:
    self._isAlignedToTarget = False

  def reset(self) -> None:
    self._drive(0.0, 0.0, 0.0)
    self.clearTargetAlignment()
  
  def _updateTelemetry(self) -> None:
    pass
  