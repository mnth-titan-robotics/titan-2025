from ntcore import NetworkTableInstance
from wpilib import Timer
import wpimath
from wpimath.geometry import Pose2d, Quaternion, Rotation2d, Rotation3d, Translation2d, Translation3d

class QuestNav:
    def __init__(self):
        nt = NetworkTableInstance.getDefault()
        nt4Table = nt.getTable("questnav")
        self._questMiso = nt4Table.getIntegerTopic("miso").subscribe(0)
        self._questMosi = nt4Table.getIntegerTopic("mosi").publish()

        self._timestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0)
        self._position = nt4Table.getDoubleArrayTopic("position").subscribe([0.0, 0.0, 0.0])
        self._quaternion = nt4Table.getDoubleArrayTopic("quaternion").subscribe([0.0, 0.0, 0.0, 0.0])
        self._eulerAngles = nt4Table.getDoubleArrayTopic("eulerAngles").subscribe([0.0, 0.0, 0.0])
        self._frameCount = nt4Table.getIntegerTopic("frameCount").subscribe(0)
        self._batteryPercent = nt4Table.getDoubleTopic("device/batteryPercent").subscribe(0.0)
        self._isTracking = nt4Table.getBooleanTopic("device/isTracking").subscribe(False)
        self._trackingLostCount = nt4Table.getIntegerTopic("device/trackingLostCounter").subscribe(0)

        self._heartbeatRequestSub = nt4Table.getDoubleTopic("heartbeat/quest_to_robot").subscribe(0.0)
        self._heartbeatResponsePub = nt4Table.getDoubleTopic("heartbeat/robot_to_quest").publish()

        self._lastProcessedHeartbeatId = 0
        self._yaw_offset = 0.0
        self._reset_position = Pose2d()

    def processHeartbeat(self):
        requestId = self._heartbeatRequestSub.get()
        if requestId > 0 and requestId != self._lastProcessedHeartbeatId:
            self._heartbeatResponsePub.set(requestId)
            self._lastProcessedHeartbeatId = requestId
    
    def getPose(self) -> Pose2d:
        compensated_pose = self._getQuestNavPose() - self._reset_position
        return Pose2d(compensated_pose.translation, Rotation2d.fromDegrees(self._getOculusYaw()))

    def getBatteryPercent(self) -> float:
        return self._batteryPercent.get()
    
    def getTrackingStatus(self) -> bool:
        return self._isTracking.get()
    
    def getFrameCount(self) -> int:
        return self._frameCount.get()
    
    def getTrackingLostCounter(self) -> int:
        return self._trackingLostCount.get()
    
    def isConnected(self) -> bool:
        # If the battery percent has been updated within the past 250ms
        return ((Timer.getFPGATimestamp() - self._batteryPercent.getLastChange()) / 1000) < 250
    
    def getQuaternion(self) -> Quaternion:
        return self._quaternion.get()
    
    def getTimestamp(self) -> float:
        return self._timestamp.getAtomic().serverTime
    
    def zeroHeading(self) -> None:
        eulerAngles = self._eulerAngles.get()
        self._yaw_offset = eulerAngles[1]
    
    def zeroPosition(self) -> None:
        self._reset_position = self.getPose()
        if self._questMiso.get() != 99:
            self._questMosi.set(1)
    
    def cleanUpQuestNavMessages(self) -> None:
        if self._questMiso.get() == 99:
            self._questMosi.set(0)
    
    def _getOculusYaw(self) -> float:
        eulerAngles = self._eulerAngles.get()
        ret = eulerAngles[1] - self._yaw_offset
        ret = ret % 360
        if ret < 0:
            ret += 360
        return ret
    
    def _getQuestNavTranslation(self) -> Translation2d:
        position = self._position.get()
        return Translation2d(position[2], -position[0])
    
    def _getQuestNavPose(self) -> Pose2d:
        # What is this magic number?
        position_compensated = self._getQuestNavTranslation() - Translation2d(0, 0.1651)
        return Pose2d(position_compensated, Rotation2d.fromDegrees(self._getOculusYaw()))

