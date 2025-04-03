from commands2 import Command, cmd
from wpilib import DriverStation, SmartDashboard, SPI, SendableChooser
from lib import logger, utils
from lib.classes import TargetAlignmentMode
from lib.controllers.game_controller import GameController
from lib.sensors.gyro_sensor_adis16470 import GyroSensor_ADIS16470
from lib.sensors.pose_sensor import PoseSensor
from core.commands.auto import AutoCommands
from core.commands.game import GameCommands
from core.subsystems.drive import DriveSubsystem
from core.subsystems.roller import RollerSubsystem
from core.subsystems.climber import ClimberSubsystem
from core.subsystems.algae_remover import AlgaeRemoverSubsystem
from core.services.localization import LocalizationService
from core.classes import TargetAlignmentLocation, TargetType
from wpimath import units
from wpilib import ADIS16470_IMU as IMU
from networktables import NetworkTables
import core.constants as constants
import wpilib
import math
#from cscore import CameraServer
## adds a simple camera server 
from wpilib.cameraserver import CameraServer

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

    # Setup Limelight & Related Variables
    # I stole this from Team Phoenix (FRC 703)
    #self._NetworkTable = NetworkTables.getTable("limelight")

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
    
  def _initSubsystems(self) -> None:
    self.driveSubsystem = DriveSubsystem(self.gyroSensor.getHeading)
    self.rollerSubsystem = RollerSubsystem()
    self.climberSubsystem = ClimberSubsystem()
    self.AlgaeRemoverSubsystem = AlgaeRemoverSubsystem()
    
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

    # Driver Controller Binds

    self.driverController.rightStick().whileTrue(self.gameCommands.alignRobotToTargetCommand(TargetAlignmentMode.Translation, TargetAlignmentLocation.Center))
    self.driverController.start().onTrue(self.gyroSensor.calibrateCommand())
    self.driverController.back().onTrue(self.gyroSensor.resetCommand())

    # Operator Controller Binds

    self.operatorController.rightTrigger().whileTrue(self.rollerSubsystem.ejectCommand())
    self.operatorController.rightBumper().whileTrue(self.climberSubsystem.climbCommand())
    self.operatorController.leftTrigger().whileTrue(self.rollerSubsystem.reverseCommand())
    self.operatorController.leftBumper().whileTrue(self.climberSubsystem.reverseCommand())
    self.operatorController.a().whileTrue(self.AlgaeRemoverSubsystem.extendCommand())
    self.operatorController.b().whileTrue(self.AlgaeRemoverSubsystem.retractCommand())

  def _periodic(self) -> None:
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

  def testInit(self) -> None:
    self.resetRobot()

  def resetRobot(self) -> None:
    self.driveSubsystem.reset()

  def _robotHasInitialZeroResets(self) -> bool:
    return utils.isCompetitionMode() or True

  def _updateTelemetry(self) -> None:
    pass
    #SmartDashboard.putBoolean("Robot/HasInitialZeroResets", self._robotHasInitialZeroResets())
