from commands2 import Subsystem, Command
import core.constants as constants
import rev

kPosition = rev.SparkBase.ControlType.kPosition

class AlgaeRemoverSubsystem(Subsystem):
    def __init__(self):
        self._motor = rev.SparkMax(constants.Subsystems.AlgaeRemover.kMotorCanId, rev.SparkBase.MotorType.kBrushless)
        self._closedLoop = self._motor.getClosedLoopController()
        self._constants = constants.Subsystems.AlgaeRemover
        config = rev.SparkMaxConfig().inverted(True).smartCurrentLimit(20)
        config.encoder.positionConversionFactor(self._constants.kPositionConversionFactor)
        config.closedLoop.pid(
            self._constants.kP,
            self._constants.kI,
            self._constants.kD
        ).outputRange(-0.5, 0.5)
        config.setIdleMode(config.IdleMode.kBrake)
        
        self._motor.configure(config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters)

    def periodic(self) -> None:
        pass
  
    def _updateTelemetry(self) -> None:
        pass

    def extendCommand(self) -> Command:
        return self.run(
            lambda: self._motor.set(constants.Subsystems.AlgaeRemover.kRotationSpeed)
        ).finallyDo(
            lambda cancel: self._motor.stopMotor()
        )
    
    def auto_extendCommand(self, set_timeout) -> Command:
        return self.run(
            lambda: self._motor.set(constants.Subsystems.AlgaeRemover.kRotationSpeed)
        ).withTimeout(set_timeout
        ).finallyDo(
            lambda cancel: self._motor.stopMotor()
        )
    
    def rampCommand(self) -> Command:
        return self.run(
            lambda: self._closedLoop.setReference(constants.Subsystems.AlgaeRemover.kRampPosition, kPosition)
        ).withName("ramp")
    
    # Are you guys not expecting to be able to remove algae while driving
    # Can't use pygame anyway because 1. you don't have it installed and 2. it's stupid considering the other code we have
    # I rewrote the code bc the other main programmer depends on ChatGPT to write code
    
    def retractCommand(self) -> Command:
        return self.run(
            lambda: self._motor.set(constants.Subsystems.AlgaeRemover.kReverseRotationSpeed)
        ).finallyDo(
            lambda cancel: self._motor.stopMotor()
        )