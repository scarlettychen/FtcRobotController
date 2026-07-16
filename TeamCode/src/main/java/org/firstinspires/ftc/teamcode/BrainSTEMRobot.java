package org.firstinspires.ftc.teamcode;

import com.pedropathing.auto.PedroBrainSTEMBridge;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.localization.ExternalPoseLocalizer;
import com.pedropathing.util.Component;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.PedroGuide;

import java.util.ArrayList;
import java.util.List;

/**
 * BrainSTEM-style robot shell.
 *
 * <p>Owns hardware, pose sync, and subsystem ticks. Does <b>not</b> construct or start
 * an auton — OpModes schedule commands or run {@code AutoMode} themselves.
 *
 * <pre>{@code
 * BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
 * RobotActions actions = PedroGuide.createActions(robot.follower);
 * scheduler.schedule(actions.driveForwardFive());
 * }</pre>
 */
public class BrainSTEMRobot {
    public final OpMode opMode;
    public final Telemetry telemetry;
    public final HardwareMap hardwareMap;

    public final ExternalPoseLocalizer pedroPoseFeed;
    public final Follower follower;
    public final PedroBrainSTEMBridge pedro;

    public boolean red;

    private final List<Component> subsystems = new ArrayList<>();

    /** Start pose (0, 0, 0) — no auton attached. */
    public BrainSTEMRobot(HardwareMap hardwareMap, Telemetry telemetry, OpMode opMode) {
        this(hardwareMap, telemetry, opMode, 0, 0, 0);
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
                new FollowerConstants(), PedroGuide.defaultMecanumConstants());
    }

    public BrainSTEMRobot(
            HardwareMap hardwareMap,
            Telemetry telemetry,
            OpMode opMode,
            double startX,
            double startY,
            double startHeadingRad,
            FollowerConstants followerConstants,
            MecanumConstants mecanumConstants
    ) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;

        pedro = PedroGuide.createBridge(
                hardwareMap,
                startX,
                startY,
                startHeadingRad,
                followerConstants,
                mecanumConstants
        );
        this.follower = pedro.getFollower();
        this.pedroPoseFeed = pedro.getPoseFeed();
    }

    /** Register a subsystem to tick after Pedro each loop. */
    public void addSubsystem(Component component) {
        if (component != null) {
            subsystems.add(component);
        }
    }

    public List<Component> getSubsystems() {
        return subsystems;
    }

    public void setAlliance(boolean red) {
        this.red = red;
    }

    /**
     * Push fused / RR pose+velocity into Pedro (inches, radians, in/s, rad/s).
     * Call every loop after your localizer updates.
     */
    public void syncPose(double x, double y, double headingRad,
                         double vx, double vy, double omega) {
        pedro.syncPoseFromRobot(x, y, headingRad, vx, vy, omega);
    }

    public void syncPose(double x, double y, double headingRad,
                         double vx, double vy, double omega,
                         double localizationConfidence) {
        pedro.syncPoseFromRobot(x, y, headingRad, vx, vy, omega, localizationConfidence);
    }

    /** Pose-only sync (zeros velocity). */
    public void syncPose(double x, double y, double headingRad) {
        pedro.syncPoseFromRobot(x, y, headingRad, 0, 0, 0);
    }

    /**
     * One robot loop tick (pose already synced). Does not run an auton —
     * call {@code auto.update()} / {@code scheduler.run()} from the OpMode.
     */
    public void update() {
        pedro.update();
        for (Component c : subsystems) {
            c.update();
        }
    }

    /** Full tick including telemetry flush. */
    public void updateWithTelemetry() {
        update();
        if (telemetry != null) {
            telemetry.update();
        }
    }

    public void reset() {
        pedro.reset();
        for (Component c : subsystems) {
            c.reset();
        }
    }
}
