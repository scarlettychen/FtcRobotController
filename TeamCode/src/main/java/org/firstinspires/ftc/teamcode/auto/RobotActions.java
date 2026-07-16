package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.auto.ActionLibrary;
import com.pedropathing.auto.AlliancePoses;
import com.pedropathing.auto.AutoCommand;
import com.pedropathing.auto.PedroDrive;

import org.firstinspires.ftc.teamcode.auto.poses.BlueClosePoses;
import org.firstinspires.ftc.teamcode.auto.poses.RedClosePoses;
import org.firstinspires.ftc.teamcode.auto.poses.TestPoses;

/**
 * Team named actions. Holds Blue + Red pose tables; call {@link #setAlliance(boolean)}
 * once at OpMode start. Prefer these over raw {@code drive.lineDrive(double[])}.
 *
 * <pre>
 * RobotActions bot = PedroGuide.createActions(robot.follower);
 * bot.setAlliance(isRed);
 * scheduler.schedule(bot.driveToCloseShoot());
 * </pre>
 */
public class RobotActions extends ActionLibrary {

    public RobotActions(PedroDrive drive) {
        this(drive, new BlueClosePoses(), new RedClosePoses());
    }

    public RobotActions(PedroDrive drive, AlliancePoses blue, AlliancePoses red) {
        super(drive, blue, red);
    }

    public RobotActions(PedroDrive drive, AlliancePoses blue, AlliancePoses red, boolean startRed) {
        super(drive, blue, red, startRed);
    }

    /** Smoke-test actions: start (0,0,0), goal at +5 in X. */
    public static RobotActions forSmokeTest(PedroDrive drive) {
        TestPoses poses = new TestPoses();
        return new RobotActions(drive, poses, poses);
    }

    // ---- Named drives (suppliers so alliance resolves at command init) ----

    /** (0,0,0) → (5,0,0) smoke-test move. */
    public AutoCommand driveForwardFive() {
        return lineDrive(() -> {
            cruise();
            AlliancePoses p = poses();
            if (p instanceof TestPoses) {
                return ((TestPoses) p).forwardFive;
            }
            return AlliancePoses.xyz(5, 0, 0);
        });
    }

    /** Drive to the close shooting / goal pose for the active alliance. */
    public AutoCommand driveToGoal() {
        return driveToCloseShoot();
    }

    public AutoCommand driveToCloseShoot() {
        return lineDrive(() -> {
            precision();
            return poses().close1Shooting;
        });
    }

    public AutoCommand driveToLookAtObelisk() {
        return lineDrive(() -> {
            cruise();
            return poses().lookAtOb;
        });
    }

    public AutoCommand driveToOpenGate() {
        return lineDrive(() -> {
            cruise();
            return poses().openGatePos;
        });
    }

    public AutoCommand driveOffLine() {
        return lineDrive(() -> {
            cruise();
            return poses().strafePos;
        });
    }

    public AutoCommand collectFirstSpike() {
        return pathDrive(
                () -> {
                    loaded();
                    return poses().collect1Pre;
                },
                () -> poses().firstSpikeEnd
        );
    }

    public AutoCommand collectSecondSpike() {
        return pathDrive(
                () -> {
                    loaded();
                    return poses().collect2Mid;
                },
                () -> poses().collect2Pre,
                () -> poses().secondSpikeEnd
        );
    }

    public AutoCommand collectThirdSpike() {
        return pathDrive(
                () -> {
                    loaded();
                    return poses().collect3Pre;
                },
                () -> poses().thirdSpikeEnd
        );
    }

    public AutoCommand driveToShootViaPass() {
        return pathDrive(
                () -> {
                    precision();
                    return poses().collect3PrePass;
                },
                () -> poses().close1Shooting
        );
    }

    // ---- Mechanism stubs (wire to BrainSTEMRobot subsystems) ----

    public AutoCommand shooterTurnOnClose() {
        return run(() -> { /* robot.shooter.setShooterShootClose(); */ });
    }

    public AutoCommand shooterIdle() {
        return run(() -> { /* robot.shooter.setShooterIdle(); */ });
    }

    public AutoCommand setCollectorOn() {
        return run(() -> { /* robot.collector.auto(); */ });
    }

    public AutoCommand setCollectorOff() {
        return run(() -> { /* robot.collector.off(); */ });
    }

    public AutoCommand rampUp() {
        return run(() -> { /* robot.ramp.up(); */ });
    }

    public AutoCommand rampDown() {
        return run(() -> { /* robot.ramp.down(); */ });
    }

    public AutoCommand moveSpindexer360() {
        return run(() -> { /* robot.spindexer.spin360(); */ });
    }
}
