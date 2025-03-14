from commands2 import Command, cmd
from wpilib import DriverStation, SmartDashboard, SPI
from lib import logger, utils
from lib.classes import TargetAlignmentMode
from lib.controllers.game_controller import GameController
from lib.sensors.gyro_sensor_adis16470 import GyroSensor_ADIS16470
from lib.sensors.pose_sensor import PoseSensor
from core.commands.auto import AutoCommands
from core.commands.game import GameCommands
from core.subsystems.drive import DriveSubsystem
from core.subsystems.roller import RollerSubsystem
from core.services.localization import LocalizationService
from core.classes import TargetAlignmentLocation, TargetType
import core.constants as constants
import wpilib
from wpimath import units
from wpilib import ADIS16470_IMU as IMU
import math

class RobotCore:
  def __init__(self) -> None:
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
    SmartDashboard.putString("Robot/Sensors/Camera/Streams", utils.toJson(constants.Sensors.Camera.kStreams))
    
  def _initSubsystems(self) -> None:
    self.driveSubsystem = DriveSubsystem(self.gyroSensor.getHeading)
    self.rollerSubsystem = RollerSubsystem()
    
  def _initServices(self) -> None:
    pass
    # self.localizationService = LocalizationService(self.gyroSensor.getRotation, self.driveSubsystem.getModulePositions, self.poseSensors)

  def _initControllers(self) -> None:
    self.driverController = GameController(constants.Controllers.kDriverControllerPort, constants.Controllers.kInputDeadband)
    self.operatorController = GameController(constants.Controllers.kOperatorControllerPort, constants.Controllers.kInputDeadband)
    DriverStation.silenceJoystickConnectionWarning(True)

  def _initCommands(self) -> None:
    self.gameCommands = GameCommands(self)
    self.autoCommands = AutoCommands(self)

  def _initTriggers(self) -> None:
    self.driveSubsystem.setDefaultCommand(
      self.driveSubsystem.driveCommand(
        lambda: - self.driverController.getLeftY(),
        self.driverController.getLeftX,
        self.driverController.getRightX
      )
    )
    self.driverController.rightStick().whileTrue(self.gameCommands.alignRobotToTargetCommand(TargetAlignmentMode.Translation, TargetAlignmentLocation.Center))
    # self.driverController.leftStick().whileTrue(cmd.none())
    #self.driverController.rightTrigger().whileTrue(self.rollerSubsystem.ejectCommand())
    # self.driverController.rightBumper().whileTrue(cmd.none())
    #self.driverController.leftTrigger().whileTrue(self.rollerSubsystem.reverseCommand())
    # self.driverController.leftBumper().whileTrue(cmd.none())
    # self.driverController.povUp().whileTrue(cmd.none())
    # self.driverController.povDown().whileTrue(cmd.none())
    # self.driverController.povLeft().whileTrue(cmd.none())
    # self.driverController.povRight().whileTrue(cmd.none())
    # self.driverController.a().whileTrue(cmd.none())
    # self.driverController.b().whileTrue(cmd.none())
    # self.driverController.y().whileTrue(cmd.none())
    # self.driverController.x().whileTrue(cmd.none())
    self.driverController.start().onTrue(self.gyroSensor.calibrateCommand())
    self.driverController.back().onTrue(self.gyroSensor.resetCommand())

    self.operatorController.rightTrigger().whileTrue(self.rollerSubsystem.ejectCommand())
    # self.operatorController.rightBumper().whileTrue(cmd.none())
    self.operatorController.leftTrigger().whileTrue(self.rollerSubsystem.reverseCommand())
    # self.operatorController.leftBumper().whileTrue(cmd.none())
    # self.operatorController.povUp().whileTrue(cmd.none())
    # self.operatorController.povDown().whileTrue(cmd.none())
    # self.operatorController.povLeft().whileTrue(cmd.none())
    # self.operatorController.povRight().whileTrue(cmd.none())
    # self.operatorController.a().whileTrue(cmd.none())
    # self.operatorController.b().whileTrue(cmd.none())
    # self.operatorController.y().whileTrue(cmd.none())
    # self.operatorController.x().whileTrue(cmd.none())
    # self.operatorController.start().whileTrue(cmd.none())
    # self.operatorController.back().whileTrue(cmd.none())

  def _periodic(self) -> None:
    self._updateTelemetry()

  def getAutoCommand(self) -> Command:
    motor_speed = 0.25

    return cmd.sequence(
      self.driveSubsystem.driveCommand(
        lambda: -(motor_speed),
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(3.25),
      self.driveSubsystem.driveCommand(
        lambda: 0.0,
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(0.1),
      self.rollerSubsystem.auto_ejectCommand(1),
      self.driveSubsystem.driveCommand(
        lambda: (motor_speed),
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(1),
      self.driveSubsystem.driveCommand(
        lambda: 0.0,
        lambda: 0.0,
        lambda: 0.0
      ).withTimeout(0.1)
    )

  def autoInit(self) -> None:
    self.resetRobot()

  def autoExit(self) -> None: 
    # self.gyroSensor.resetRobotToField(self.localizationService.getRobotPose())
    pass

  def teleopInit(self) -> None:
    self.resetRobot()

  def testInit(self) -> None:
    self.resetRobot()

  def resetRobot(self) -> None:
    self.driveSubsystem.reset()

  def _robotHasInitialZeroResets(self) -> bool:
    return utils.isCompetitionMode() or True

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/HasInitialZeroResets", self._robotHasInitialZeroResets())
