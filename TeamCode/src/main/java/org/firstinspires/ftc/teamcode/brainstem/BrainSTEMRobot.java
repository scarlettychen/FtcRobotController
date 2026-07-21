package org.firstinspires.ftc.teamcode.brainstem;

import com.pedropathing.auto.AlliancePoses;
import com.pedropathing.auto.PedroBrainSTEMBridge;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.ExternalPoseLocalizer;
import com.pedropathing.util.Component;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Team-owned BrainSTEM robot shell.
 *
 * <p>Owns hardware, Pinpoint odometry, model, pose synchronization, and subsystem ticks.
 * It never creates, schedules, starts, or stops an auton; the OpMode owns that lifecycle.
 *
 * <p>Each {@link #update()} reads Pinpoint and pushes pose/velocity into Pedro's
 * {@link ExternalPoseLocalizer}, which is how the follower knows how far the robot traveled.
 */
public class BrainSTEMRobot {
    public final OpMode opMode;
    public final Telemetry telemetry;
    public final HardwareMap hardwareMap;

    public final RobotConfiguration configuration;
    public final RobotModel robotModel;
    public final PinpointLocalizer pinpoint;
    public final ExternalPoseLocalizer pedroPoseFeed;
    public final Follower follower;
    public final PedroBrainSTEMBridge pedro;

    public boolean red;

    private final List<Component> subsystems = new ArrayList<>();

    public BrainSTEMRobot(HardwareMap hardwareMap, Telemetry telemetry, OpMode opMode) {
        this(hardwareMap, telemetry, opMode, 0, 0, 0, new RobotConfiguration());
    }

    public BrainSTEMRobot(
            HardwareMap hardwareMap,
            Telemetry telemetry,
            OpMode opMode,
            double startX,
            double startY,
            double startHeadingRad
    ) {
        this(hardwareMap, telemetry, opMode, startX, startY, startHeadingRad,
                new RobotConfiguration());
    }

    public BrainSTEMRobot(
            HardwareMap hardwareMap,
            Telemetry telemetry,
            OpMode opMode,
            double startX,
            double startY,
            double startHeadingRad,
            RobotConfiguration configuration
    ) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;
        this.configuration = configuration;
        this.robotModel = configuration.createRobotModel();

        pedro = PedroGuide.createBridge(
                hardwareMap,
                startX,
                startY,
                startHeadingRad,
                configuration,
                robotModel
        );
        follower = pedro.getFollower();
        pedroPoseFeed = pedro.getPoseFeed();

        Pose startPose = Pose.fromField(startX, startY, startHeadingRad);
        pinpoint = new PinpointLocalizer(
                hardwareMap,
                configuration.createPinpointConstants(),
                startPose
        );
        syncPinpointIntoPedro();
    }

    public void addSubsystem(Component component) {
        if (component != null) {
            subsystems.add(component);
        }
    }

    public List<Component> getSubsystems() {
        return Collections.unmodifiableList(subsystems);
    }

    public void setAlliance(boolean red) {
        this.red = red;
    }

    /**
     * Apply an auton-specific start pose ({@code x, y, headingDegrees}) before start.
     * Updates Pinpoint and Pedro's pose feed together.
     *
     * <p>Uses {@link PinpointLocalizer#setPose} (absolute), not {@code setStartPose}. Pinpoint's
     * {@code setStartPose} rebases relative to the previous start and corrupts XY when called
     * after construction (e.g. {@code (72,72)} → {@code (216,72)}).
     */
    public void setStartPose(double[] startPose) {
        Pose pose = AlliancePoses.toPose(startPose);
        // Absolute stamp — do not call pinpoint.setStartPose (rebase math).
        pinpoint.setPose(pose);
        pedroPoseFeed.setStartPose(pose);
        follower.setStartingPose(pose);
        syncPinpointIntoPedro();
        if (follower.getPoseTracker() != null) {
            follower.getPoseTracker().invalidateCache();
        }
    }

    public void syncPose(double x, double y, double headingRad,
                         double vx, double vy, double omega) {
        pedro.syncPoseFromRobot(x, y, headingRad, vx, vy, omega);
    }

    public void syncPose(double x, double y, double headingRad,
                         double vx, double vy, double omega,
                         double localizationConfidence) {
        pedro.syncPoseFromRobot(
                x, y, headingRad, vx, vy, omega, localizationConfidence);
    }

    public void syncPose(double x, double y, double headingRad) {
        pedro.syncPoseFromRobot(x, y, headingRad, 0, 0, 0);
    }

    /**
     * One robot loop tick: Pinpoint → Pedro pose feed → bridge/subsystems.
     * Call this every OpMode loop while following.
     */
    public void update() {
        pinpoint.update();
        syncPinpointIntoPedro();
        // Pinpoint wrote ExternalPoseLocalizer outside PoseTracker.update(); drop the
        // cached pose so path baking / following see the live Pinpoint heading/XY.
        if (follower != null && follower.getPoseTracker() != null) {
            follower.getPoseTracker().invalidateCache();
        }
        pedro.update();
        for (Component component : subsystems) {
            component.update();
        }
    }

    public void updateWithTelemetry() {
        update();
        if (telemetry != null) {
            Pose pose = pinpoint.getPose();
            telemetry.addData("pinpoint x", pose.getX());
            telemetry.addData("pinpoint y", pose.getY());
            telemetry.addData("pinpoint h deg", Math.toDegrees(pose.getHeading()));
            telemetry.update();
        }
    }

    public void reset() {
        pedro.reset();
        for (Component component : subsystems) {
            component.reset();
        }
    }

    /** Copy Pinpoint's Pedro-frame pose/velocity into the ExternalPoseLocalizer. */
    private void syncPinpointIntoPedro() {
        Pose pose = pinpoint.getPose();
        Pose velocity = pinpoint.getVelocity();
        pedroPoseFeed.setPedroState(
                pose.getX(),
                pose.getY(),
                pose.getHeading(),
                velocity.getX(),
                velocity.getY(),
                velocity.getHeading()
        );
    }
}
