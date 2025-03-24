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
from core.subsystems.climber import ClimberSubsystem
from core.services.localization import LocalizationService
from core.classes import TargetAlignmentLocation, TargetType
import core.constants as constants
import wpilib
from wpimath import units
from wpilib import ADIS16470_IMU as IMU
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

    #CameraServer.enableLogging()
    #self.camera = CameraServer.startAutomaticCapture()
    #self.camera.setResolution(160, 160)
    #self.camera.setFPS(60)
    #self.camera_output = CameraServer.putVideo("Driver_Camera", 160, 160)


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
    #SmartDashboard.putString("Robot/Sensors/Camera/Streams", utils.toJson(constants.Sensors.Camera.kStreams))
    
  def _initSubsystems(self) -> None:
    self.driveSubsystem = DriveSubsystem(self.gyroSensor.getHeading)
    self.rollerSubsystem = RollerSubsystem()
    self.climberSubsystem = ClimberSubsystem()
    
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
    self.operatorController.rightBumper().whileTrue(self.climberSubsystem.climbCommand())
    self.operatorController.leftTrigger().whileTrue(self.rollerSubsystem.reverseCommand())
    self.operatorController.leftBumper().whileTrue(self.climberSubsystem.reverseCommand())
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
    #CameraServer.putVideo("Driver_Camera", 160, 160)

  def getAutoCommand(self) -> Command:
    motor_speed = 0.25
    motor_stop_time = 0.1

    mode_0_drive_time = 3.25
    other_mode_step_1_drive_time = 1.85
    other_mode_step_2_drive_time = 1.75

    robot_turn_angle = 45.0

    # Change the value of the "selected_mode" variable to change modes, then redeploy
    selected_mode = "right"
    """
      # The allowed modes are "center", "left", "right"
      # DO NOT CAPITALIZE ANY OF THE MODES
      #
      # The modes are assuming the robot is coming from the barge, driving towards the reef
      # 
      # "center" drives straight forward, into the reef, then dispensing the coral without turning
      # "left" drives straight forward, turning right, towards the reef, 
      #       then driving straight forward into the reef, then dispensing the coral
      # "right" drives straight forward, turning left, towards the reef,
      #       then driving straight forward into the reef, then dispensing the coral
    """

    selected_mode.lower()

    if selected_mode == "center":
      # This is for driving straight then dispensing the coral
      return cmd.sequence(
        self.driveSubsystem.driveCommand(
          lambda: -(motor_speed),
          lambda: 0.0,
          lambda: 0.0
        ).withTimeout(mode_0_drive_time),
        self.driveSubsystem.driveCommand(
          lambda: 0.0,
          lambda: 0.0,
          lambda: 0.0
        ).withTimeout(motor_stop_time),
        self.rollerSubsystem.auto_ejectCommand(1)
      )
    
    elif selected_mode in ["left", "right"]:
      # This is for driving straight, turning a certain direction, 
      #     driving straight again before dispensing the coral
      return cmd.sequence(
        self.driveSubsystem.driveCommand( # Drive forward beside reef
          lambda: -(motor_speed),
          lambda: 0.0,
          lambda: 0.0
        ).withTimeout(other_mode_step_1_drive_time),
        self.driveSubsystem.driveCommand( # Stop driving
          lambda: 0.0,
          lambda: 0.0,
          lambda: 0.0
        ).withTimeout(motor_stop_time),
        self.driveSubsystem.driveCommand( # Turn to face reef
          lambda: 0.0,
          lambda: 0.0,    # The line below determines whether or not we are turning left or right
          lambda: (robot_turn_angle) if selected_mode == "right" else -(robot_turn_angle) # One-liner for changing signs
        ).withTimeout(0.2),
        self.driveSubsystem.driveCommand( # Stop turning
          lambda: 0.0,
          lambda: 0.0,
          lambda: 0.0
        ).withTimeout(motor_stop_time),
        self.driveSubsystem.driveCommand( # Drive forward to reef
          lambda: -(motor_speed),
          lambda: 0.0,
          lambda: 0.0
        ).withTimeout(other_mode_step_2_drive_time),
        self.driveSubsystem.driveCommand( # Stop driving
          lambda: 0.0,
          lambda: 0.0,
          lambda: 0.0
        ).withTimeout(motor_stop_time),
        self.rollerSubsystem.auto_ejectCommand(1)
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
    pass
    #SmartDashboard.putBoolean("Robot/HasInitialZeroResets", self._robotHasInitialZeroResets())
