from commands2 import Command
from wpilib import DriverStation
from lib import utils
from lib.classes import TargetAlignmentMode
from lib.controllers.game_controller import GameController
from lib.sensors.gyro_sensor_adis16470 import GyroSensor_ADIS16470
from lib.sensors.pose_sensor import PoseSensor
from core.commands.auto import AutoCommands
from core.commands.game import GameCommands
from core.subsystems.drive import DriveSubsystem
from core.subsystems.roller import RollerSubsystem
from core.subsystems.Baby_Roller import Baby_RollerSubsystem
from core.subsystems.climber import ClimberSubsystem
from core.subsystems.algae_remover import AlgaeRemoverSubsystem
from core.classes import TargetAlignmentLocation
from wpilib import ADIS16470_IMU as IMU
from wpimath.geometry import Transform3d, Pose3d
from ntcore import NetworkTableInstance
from core.services.mecanum_localization import LocalizationService


import core.constants as constants
import wpilib
import photonlibpy
import math


class RobotCore(wpilib.TimedRobot):
  def robotInit(self):
   pass

  def __init__(self) -> None:
    super().__init__()

    self._initSensors()
    self._initSubsystems()
    self._initServices()
    self._initControllers()
    self._initCommands()
    self._initTriggers()
    utils.addRobotPeriodic(self._periodic)

  def _initSensors(self) -> None:
    self.gyroSensor = GyroSensor_ADIS16470(
      wpilib.SPI.Port.kMXP, 
      imuAxisRoll=IMU.IMUAxis.kZ, 
      imuAxisPitch=IMU.IMUAxis.kX, 
      imuAxisYaw=IMU.IMUAxis.kY, 
      initCalibrationTime=IMU.CalibrationTime._1s, 
      commandCalibrationTime= IMU.CalibrationTime._1s, 
      commandCalibrationDelay=1.0
    )

    self.poseSensors = tuple(PoseSensor(c) for c in constants.Sensors.Pose.kPoseSensorConfigs)

    self._photonCamera = photonlibpy.PhotonCamera("Arducam_OV9281_USB_Camera")

    self._photon_rot = None
    self._photon_xValue = None

    self._photon_estimator = photonlibpy.PhotonPoseEstimator(
            constants.APRIL_TAG_FIELD_LAYOUT, 
            constants.Sensors.Pose._poseStrategy,
            self._photonCamera,
            Transform3d()
    )
    
  def _initSubsystems(self) -> None:
    self.driveSubsystem = DriveSubsystem(self.gyroSensor.getHeading)
    self.rollerSubsystem = RollerSubsystem()
    self.climberSubsystem = ClimberSubsystem()
    self.AlgaeRemoverSubsystem = AlgaeRemoverSubsystem()
    self.Baby_RollerSubsystem = Baby_RollerSubsystem ()

  def _initServices(self) -> None:    
    nt = NetworkTableInstance.getDefault()
    self._targetVisible = nt.getBooleanTopic("Vision/TargetVisible").publish()
    self._aprilTag = nt.getIntegerTopic("Vision/AprilTag").publish()
    self._targetYaw = nt.getDoubleTopic("Vision/TargetYaw").publish()
    self._targetRange = nt.getDoubleTopic("Vision/TargetRange").publish()
    
    self.localizationService = LocalizationService(self.gyroSensor.getRotation, self.driveSubsystem.getModulePositions, self.poseSensors)

  def _initControllers(self) -> None:
    self.driverController = GameController(constants.Controllers.kDriverControllerPort, constants.Controllers.kInputDeadband)
    self.operatorController = GameController(constants.Controllers.kOperatorControllerPort, constants.Controllers.kInputDeadband)
    DriverStation.silenceJoystickConnectionWarning(True)

  def _initCommands(self) -> None:
    self.gameCommands = GameCommands(self)
    self.autoCommands = AutoCommands(self)

  def _driveLockOn(self):
    if self._photon_rot and self._photon_xValue:
      self.driveSubsystem._drive(self._photon_xValue, 0, self._photon_rot)

  def _lockOnTag(self) -> Command:
    return self.driveSubsystem.run(
      self._driveLockOn
    )

  def _initTriggers(self) -> None:
    self.driveSubsystem.setDefaultCommand(
      self.driveSubsystem.driveCommand(
        lambda: - self.driverController.getLeftY(),
        self.driverController.getLeftX,
        self.driverController.getRightX
      )
    )

    # Driver Controller Binds
    self.driverController.rightStick().whileTrue(self.gameCommands.alignRobotToTargetCommand(TargetAlignmentMode.Translation, TargetAlignmentLocation.Center))
    self.driverController.start().onTrue(self.gyroSensor.calibrateCommand())
    self.driverController.back().onTrue(self.gyroSensor.resetCommand())
    self.driverController.rightBumper().whileTrue(self._lockOnTag())

    # Operator Controller Binds
    self.operatorController.rightTrigger().whileTrue(self.rollerSubsystem.ejectCommand())
    self.operatorController.rightBumper().whileTrue(self.climberSubsystem.climbCommand())
    self.operatorController.leftTrigger().whileTrue(self.rollerSubsystem.reverseCommand())
    self.operatorController.leftBumper().whileTrue(self.climberSubsystem.reverseCommand())
    self.operatorController.a().whileTrue(self.AlgaeRemoverSubsystem.extendCommand())
    self.operatorController.b().whileTrue(self.AlgaeRemoverSubsystem.retractCommand())
    self.operatorController.x().whileTrue(self.Baby_RollerSubsystem.reverseCommand())
    self.operatorController.y().whileTrue(self.Baby_RollerSubsystem.intakeCommand())
    self.operatorController.povUp().whileTrue(self.AlgaeRemoverSubsystem.rampCommand())

  def _periodic(self) -> None:
    self.localizationService._periodic()
    self.teleopPeriodic()
    self._updateTelemetry()

  def getAutoCommand(self) -> Command:
    return self.autoCommands.getSelected()

  def autoInit(self) -> None:
    self.resetRobot()

  def autoExit(self) -> None: 
    # self.gyroSensor.resetRobotToField(self.localizationService.getRobotPose())
    pass

  def teleopInit(self) -> None:
    self.resetRobot()
  
  def teleopPeriodic(self):
    # We stole this from photonvision docs "Combining Aiming and Getting in Range"
    xSpeed = self.driverController.getLeftY() #-1.0 * self.driverController.getLeftY() * constants.Subsystems.Drive.kRotationSpeedMax
    ySpeed = self.driverController.getLeftX() #-1.0 * self.driverController.getLeftX() * constants.Subsystems.Drive.kRotationSpeedMax
    rot = -1.0 * self.driverController.getRightX() * constants.Subsystems.Drive.kTranslationSpeedMax

    VISION_TURN_kP = 0.01
    VISION_DES_ANGLE_deg = 0.0 # Degree
    VISION_STRAFE_kP = 0.5
    VISION_DES_RANGE_m = 0.5 # Meters

    CAM_MOUNT_HEIGHT_m = 0.6223 # Meters
    TAG_MOUNT_HEIGHT_m = 0.254 # Meters
    CAM_MOUNT_PITCH_deg = 0.0 # Degrees

    # Get information from the camera
    targetYaw = 0.0
    targetRange = 0.0
    targetVisible = False
    aprilTag = 0
    
    if self._photonCamera.isConnected():
      results = self._photonCamera.getAllUnreadResults()

      if len(results) > 0:
        result = results[-1]  # take the most recent result the camera had

        # At least one apriltag was seen by the camera
        for target in result.getTargets():
          if target.getFiducialId() in [6,7,8,9,10,11,17,18,19,20,21,22]: # These are the Apriltag IDs for the reef positions
            # Found tag, record its information
            targetVisible = True

            aprilTag = target.getFiducialId()
            targetYaw = target.getYaw()
            heightDelta = CAM_MOUNT_HEIGHT_m - TAG_MOUNT_HEIGHT_m
            angleDelta = math.radians(CAM_MOUNT_PITCH_deg - target.getPitch())
            targetRange = heightDelta / math.tan(angleDelta)
            # print("Found Target With Variables:")
            # print(f"Target Visible: {targetVisible}\nTargetYaw: {targetYaw}\nHeightDelta: {heightDelta}")
            # print(f"AngleDelta: {angleDelta}\nTargetRange: {targetRange}")
            break
      
      self._targetVisible.set(targetVisible)
      self._targetYaw.set(targetYaw)
      self._targetRange.set(targetRange)
      self._aprilTag.set(aprilTag)

      if targetVisible:
        # Driver wants auto-alignment to tag
        # And, tag is in sight, so we can turn toward it.
        # Override the driver's turn and x-vel command with
        # an automatic one that turns toward the tag
        # and puts us at the right distance

        self._photon_rot = (
          (VISION_DES_ANGLE_deg - targetYaw)
          * VISION_TURN_kP
          * constants.Subsystems.Drive.kTranslationSpeedMax
        )
        self._photon_xValue = (
          (VISION_DES_RANGE_m - targetRange)
          * VISION_STRAFE_kP
          * constants.Subsystems.Drive.kRotationSpeedMax
        )

  def testInit(self) -> None:
    self.resetRobot()

  def resetRobot(self) -> None:
    self.driveSubsystem.reset()

  def _robotHasInitialZeroResets(self) -> bool:
    return utils.isCompetitionMode() or True

  def _updateTelemetry(self) -> None:
    pass
    #SmartDashboard.putBoolean("Robot/HasInitialZeroResets", self._robotHasInitialZeroResets())
