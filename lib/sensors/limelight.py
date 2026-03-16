from ntcore import NetworkTableInstance
from wpimath.geometry import Pose3d, Pose2d, Translation3d, Rotation3d, Rotation2d, Translation2d
from core.classes import units
from wpilib import SmartDashboard
import math


class PoseEstimate:
        pose = Pose2d()
        timestampSeconds = 0
        latency = 0
        tagCount = 0
        tagSpan = 0
        avgTagDist = 0
        avgTagArea = 0
        rawFiducials = []
        isMegaTag2 = False

        def __init__(
            self, pose, timestampSeconds, latency, tagCount, 
            tagSpan, avgTagDist, avgTagArea, rawFiducials, isMegaTag2):
            self.pose = pose
            self.timestampSeconds = timestampSeconds
            self.latency = latency
            self.tagCount = tagCount
            self.tagSpan = tagSpan
            self.avgTagDist = avgTagDist
            self.avgTagArea = avgTagArea
            self.rawFiducials = rawFiducials
            self.isMegaTag2 = isMegaTag2
        
        def equals(self, obj):
            if self == obj:
                return True
            if (obj == None) or (self.__class__ != obj.__class__):
                return False
            that: PoseEstimate = obj
            # We don't compare the timestampSeconds as it isn't relevant for equality and makes
            # unit testing harder
            return (
                math.isclose(that.latency, self.latency) and
                math.isclose(self.tagCount, that.tagCount) and
                math.isclose(that.tagSpan, self.tagSpan) and
                math.isclose(that.avgTagDist, self.avgTagDist) and
                math.isclose(that.avgTagArea, self.avgTagArea) and
                (self.pose == that.pose) and
                (self.rawFiducials == that.rawFiducials)
            )


class RawFiducial:
    id = 0
    txnc = 0
    tync = 0
    ta = 0
    distToCamera = 0
    distToRobot = 0
    ambiguity = 0

    def __init__(self, id, txnc, tync, ta, distToCamera, distToRobot, ambiguity):
        self.id = id
        self.txnc = txnc
        self.tync = tync
        self.ta = ta
        self.distToCamera = distToCamera
        self.distToRobot = distToRobot
        self.ambiguity = ambiguity

    def equals(self, obj):
        if self == obj:
            return True
        if (obj == None) or (self.__class__ != obj.__class__):
            return False
        other: RawFiducial = obj # RawFiducial other = (RawFiducial) obj;
        return (
            self.id == other.id and
            math.isclose(self.txnc, other.txnc) and
            math.isclose(self.tync, other.tync) and
            math.isclose(self.ta, other.ta) and
            math.isclose(self.distToCamera, other.distToCamera) and
            math.isclose(self.distToRobot, other.distToRobot) and
            math.isclose(self.ambiguity, other.ambiguity)
        )


class LimeLightHelper:
    def __init__(self):
        nt_instance = NetworkTableInstance.getDefault()
        self._limelight_name = "limelight"
        self._table = nt_instance.getTable(self._limelight_name)

        self._botpose_topic = self._table.getDoubleArrayTopic("botpose").subscribe([])
        self._botpose_wpiblue_topic = self._table.getDoubleArrayTopic("botpose_wpiblue").subscribe([])

    def getPose3d(self) -> Pose3d:
        values = self._botpose_topic.get()
        if len(values) >= 6:
            return Pose3d(
                Translation3d(values[0], values[1], values[2]),
                Rotation3d.fromDegrees(values[3], values[4], values[5])
            )
        else:
            return Pose3d()
    
    def getTV(self) -> bool:
        #return self._table.getDoubleTopic("tv").getEntry(0.0) == 1.0
        return False

    def getTX(self) -> float:
        #return self._table.getDoubleTopic("tx").getEntry(0.0)
        return 0

    def getTY(self) -> float:
        #return self._table.getDoubleTopic("ty").getEntry(0.0)
        return 0
    
    def updateTelemetry(self):
        SmartDashboard.putBoolean("TV", self.getTV())
        SmartDashboard.putNumber("TX", self.getTX())
        SmartDashboard.putNumber("TY", self.getTY())
    
    def toPose2d(self, data) -> Pose2d:
        if len(data) < 6:
            return Pose2d()
        else:
            tran2d = Translation2d(data[0], data[1])
            r2d = Rotation2d(units.degreesToRadians(data[5]))
            return Pose2d(tran2d, r2d)
    
    def getBotPoseEstimate_wpiRed_MegaTag2(self):
        return self.getBotPoseEstimate(self._limelight_name, "botpose_orb_wpired", True)
    
    def extractArrayEntry(self, inData, position):
        if len(inData) < (position + 1):
            return 0
        return inData[position]

    def getBotPoseEstimate(self, limelightName, entryName, isMegaTag2):
        poseEntry = self._botpose_wpiblue_topic.get([])

        tsValue = poseEntry.getAtomic()
        poseArray = tsValue.value
        timestamp = tsValue.serverTime

        if len(poseArray) == 0:
            return None # maybe change
        
        pose = Pose2d(poseArray)
        latency = self.extractArrayEntry(poseArray, 6)
        tagCount = self.extractArrayEntry(poseArray, 7)
        tagSpan = self.extractArrayEntry(poseArray, 8)
        tagDist = self.extractArrayEntry(poseArray, 9)
        tagArea = self.extractArrayEntry(poseArray, 10)

        # Convert server timestamp from microseconds to seconds and adjust for latency
        adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0)

        rawFiducials = []
        valsPerFiducial = 7
        expectedTotalVals = (11 + valsPerFiducial * tagCount)

        if len(poseArray) == expectedTotalVals:
            for i in range(tagCount):
                baseIndex = (11 + (i * valsPerFiducial))
                id = poseArray[baseIndex]
                txnc = poseArray[baseIndex + 1]
                tync = poseArray[baseIndex + 2]
                ta = poseArray[baseIndex + 3]
                distToCamera = poseArray[baseIndex + 4]
                distToRobot = poseArray[baseIndex + 5]
                ambiguity = poseArray[baseIndex + 6]
                rawFiducials.append(
                    RawFiducial(
                        id, 
                        txnc, 
                        tync, 
                        ta, 
                        distToCamera, 
                        distToRobot, 
                        ambiguity
                    )
                )
        return PoseEstimate(
            pose, 
            adjustedTimestamp, 
            latency, 
            tagCount, 
            tagSpan, 
            tagDist, 
            tagArea, 
            rawFiducials, 
            isMegaTag2
        )