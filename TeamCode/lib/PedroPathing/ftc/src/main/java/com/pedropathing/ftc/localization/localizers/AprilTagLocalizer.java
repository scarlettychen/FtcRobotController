package com.pedropathing.ftc.localization.localizers;

import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Limelight AprilTag botpose → field pose (inches) for fusion with odometry.
 */
public class AprilTagLocalizer {
    private static final double METERS_TO_INCHES = 39.3701;

    private final Limelight3A limelight;
    private Pose3D lastBotpose;

    public static final class VisionMeasurement {
        public final Pose pose;
        public final double distanceToTag;
        public final long timestamp;

        public VisionMeasurement(Pose pose, double distanceToTag, long timestamp) {
            this.pose = pose;
            this.distanceToTag = distanceToTag;
            this.timestamp = timestamp;
        }
    }

    public AprilTagLocalizer(HardwareMap hwMap) {
        this(hwMap, "limelight", 0);
    }

    public AprilTagLocalizer(HardwareMap hwMap, String deviceName, int pipeline) {
        this.limelight = hwMap.get(Limelight3A.class, deviceName);
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public VisionMeasurement getLatestMeasurement() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        Pose3D botpose = result.getBotpose();
        if (botpose == null) return null;

        if (lastBotpose != null
                && botpose.getPosition().x == lastBotpose.getPosition().x
                && botpose.getPosition().y == lastBotpose.getPosition().y) {
            return null;
        }
        lastBotpose = botpose;

        double xInches = botpose.getPosition().x * METERS_TO_INCHES;
        double yInches = botpose.getPosition().y * METERS_TO_INCHES;
        double heading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
        Pose robotPose = PoseConverter.fromField(xInches, yInches, heading);

        double minDistance = Double.MAX_VALUE;
        List<FiducialResult> fiducials = result.getFiducialResults();
        for (FiducialResult fiducial : fiducials) {
            double distanceInches =
                    Math.abs(fiducial.getCameraPoseTargetSpace().getPosition().z * METERS_TO_INCHES);
            if (distanceInches < minDistance) minDistance = distanceInches;
        }
        if (minDistance == Double.MAX_VALUE) minDistance = 0;

        return new VisionMeasurement(robotPose, minDistance, System.nanoTime());
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}
