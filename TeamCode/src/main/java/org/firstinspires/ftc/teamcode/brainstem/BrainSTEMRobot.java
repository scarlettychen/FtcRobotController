package org.firstinspires.ftc.teamcode.brainstem;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.ExternalPoseLocalizer;
import com.pedropathing.util.Component;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.brainstem.follower.PathFollower;
import org.firstinspires.ftc.teamcode.brainstem.follower.PedroFollowerAdapter;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Drive;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.FourBarLinkage;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Intake;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.IntakeBeamBreak;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.brainstem.utils.BatteryVoltageFilter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

// the big robot class. hardware + pinpoint + pedro + subsystems
// make one of these in ur opmode, call update() every loop
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

    public final Intake intake;
    public final Transfer transfer;
    public final FourBarLinkage lift;
    public final Blocker blocker;
    public final Limelight limelight;
    public final IntakeBeamBreak intakeGate;
    public final Drive drive;

    public boolean red;

    private final List<Component> subsystems = new ArrayList<>();
    private final BatteryVoltageFilter batteryFilter;

    public BrainSTEMRobot(HardwareMap hardwareMap, Telemetry telemetry, OpMode opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;
        this.configuration = new RobotConfiguration();
        this.robotModel = configuration.createRobotModel();

        pedro = PedroGuide.createBridge(hardwareMap, configuration, robotModel);
        follower = pedro.getFollower();
        pedroPoseFeed = pedro.getPoseFeed();

        Pose origin = Pose.fromField(0, 0, 0);
        pinpoint = new PinpointLocalizer(
                hardwareMap,
                configuration.createPinpointConstants(),
                origin
        );
        syncPinpointIntoPedro();

        batteryFilter = new BatteryVoltageFilter(hardwareMap);

        intake = new Intake(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);
        lift = new FourBarLinkage(hardwareMap, telemetry);
        blocker = new Blocker(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);
        intakeGate = new IntakeBeamBreak(hardwareMap, telemetry);
        drive = new Drive(hardwareMap, configuration);
        addSubsystem(intake);
        addSubsystem(transfer);
        addSubsystem(lift);
        addSubsystem(blocker);
        addSubsystem(limelight);
        addSubsystem(intakeGate);
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

    /** Classic Pedro follower behind the portable {@link PathFollower} contract. */
    public PathFollower createPathFollower() {
        return new PedroFollowerAdapter(follower, robotModel);
    }

    /** Field pose {@code {x, y, headingDeg}} in {@link RoadRunnerCoordinates} / FieldCoords. */
    public double[] getFieldPose() {
        Pose field = pinpoint.getPose().getAsCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
        return new double[]{
                field.getX(),
                field.getY(),
                Math.toDegrees(field.getHeading())
        };
    }

    // set match start as {x, y, headingDeg} in FieldCoords (0°=+Y into field, CCW+).
    // skip on simple drive-forward tests if you want raw pedro (0,0,0).
    public void setStartPose(double[] startPose) {
        Pose pose = Pose.fromFieldDegrees(
                startPose[0],
                startPose[1],
                startPose.length >= 3 ? startPose[2] : 0);
        // hard reset — Pinpoint.setStartPose() rebases and can corrupt XY if called twice
        pinpoint.resetPose(pose);
        pedroPoseFeed.setStartPose(pose);
        follower.setStartingPose(pose);
        syncPinpointIntoPedro();
        if (follower.getPoseTracker() != null) {
            follower.getPoseTracker().invalidateCache();
        }
    }

    // one loop: pinpoint → pedro → subsystems
    public void update() {
        pinpoint.update();
        syncPinpointIntoPedro();
        if (follower.getPoseTracker() != null) {
            follower.getPoseTracker().invalidateCache();
        }

        batteryFilter.update();
        robotModel.setBatteryVoltage(batteryFilter.getVoltage());

        pedro.update();

        for (Component component : subsystems) {
            component.update();
        }
    }

    public void reset() {
        pedro.reset();
        for (Component component : subsystems) {
            component.reset();
        }
    }

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
