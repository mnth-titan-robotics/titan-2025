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
from core.subsystems.climber import ClimberSubsystem
from core.subsystems.algae_remover import AlgaeRemoverSubsystem
from core.classes import TargetAlignmentLocation
from wpilib import ADIS16470_IMU as IMU
import core.constants as constants
import wpilib
import photonlibpy


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

    self._alignToApriltagToggle = False

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
    self._alignToApriltagToggle = self.operatorController.x().getAsBoolean()

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
  
  def teleopPeriodic(self) -> None:
    self.rollerSubsystem.auto_ejectCommand(0.25)

    # We stole this from photonvision docs "Aiming at Target"
    xSpeed = -1.0 * self.driverController.getLeftY() * constants.Subsystems.Drive.kRotationSpeedMax
    ySpeed = -1.0 * self.driverController.getLeftX() * constants.Subsystems.Drive.kRotationSpeedMax
    rot = -1.0 * self.driverController.getRightX() * constants.Subsystems.Drive.kRotationSpeedMax

    # Get information from the camera
    targetYaw = 0.0
    targetVisible = False
    results = self._photonCamera.getAllUnreadResults()
    if len(results) > 0:
        result = results[-1]  # take the most recent result the camera had
        for target in result.getTargets():
            if target.getFiducialId() in [6,7,8,9,10,11,17,18,19,20,21,22]:
                # Found tag 7, record its information
                # We only care if we found an AprilTag that's on the reef
                targetVisible = True
                targetYaw = target.getYaw()
                print(f"Found AprilTag {target.getFiducialId()}")

    if self._alignToApriltagToggle and targetVisible:
        # Driver wants auto-alignment to tag 7
        # And, tag 7 is in sight, so we can turn toward it.
        # Override the driver's turn command with an automatic one that turns toward the tag.
        rot = -1.0 * targetYaw * (0.25) * constants.Subsystems.Drive.kRotationSpeedMax

    self.driveSubsystem._drive(xSpeed, ySpeed, rot)
    print("Attempted to Drive")

  def testInit(self) -> None:
    self.resetRobot()

  def resetRobot(self) -> None:
    self.driveSubsystem.reset()

  def _robotHasInitialZeroResets(self) -> bool:
    return utils.isCompetitionMode() or True

  def _updateTelemetry(self) -> None:
    pass
    #SmartDashboard.putBoolean("Robot/HasInitialZeroResets", self._robotHasInitialZeroResets())
