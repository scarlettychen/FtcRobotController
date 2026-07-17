package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.auto.ActionLibrary;
import com.pedropathing.auto.AlliancePoses;
import com.pedropathing.auto.AutoCommand;
import com.pedropathing.auto.PedroDrive;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.auto.poses.BlueClosePoses;
import org.firstinspires.ftc.teamcode.brainstem.auto.poses.RedClosePoses;
import org.firstinspires.ftc.teamcode.brainstem.auto.poses.RobotPoses;
import org.firstinspires.ftc.teamcode.brainstem.auto.poses.TestPoses;

/**
 * Team-editable named drive and subsystem commands.
 *
 * <p>Add new autonomous actions here. Public auto routines should call names such as
 * {@link #driveToGoal()} instead of raw coordinate/path builders.
 */
public class RobotActions extends ActionLibrary {
    protected final BrainSTEMRobot robot;
    private final RobotPoses bluePoses;
    private final RobotPoses redPoses;

    public RobotActions(BrainSTEMRobot robot) {
        this(robot, new PedroDrive(robot.follower), new BlueClosePoses(), new RedClosePoses());
    }

    public RobotActions(
            BrainSTEMRobot robot,
            PedroDrive drive,
            RobotPoses blue,
            RobotPoses red
    ) {
        super(drive, blue, red);
        this.robot = robot;
        this.bluePoses = blue;
        this.redPoses = red;
    }

    /** Smoke-test actions: start (0,0,0), goal at +5 in X. */
    public static RobotActions forSmokeTest(BrainSTEMRobot robot) {
        TestPoses poses = new TestPoses();
        return new RobotActions(robot, new PedroDrive(robot.follower), poses, poses);
    }

    // ---- Named drives: coordinates resolve when each command initializes ----

    public AutoCommand driveForwardFive() {
        cruise();
        return getDrive().forwardDrive(5.0);
    }
    public AutoCommand side() {
        cruise();
        return getDrive().turnTo(90);
    }


    public AutoCommand driveToGoal() {
        return driveToCloseShoot();
    }

    public AutoCommand driveToCloseShoot() {
        return lineDrive(() -> {
            cruise();
            return robotPoses().close1Shooting;
        });
    }

    public AutoCommand driveToLookAtObelisk() {
        return lineDrive(() -> {
            cruise();
            return robotPoses().lookAtOb;
        });
    }

    public AutoCommand driveToOpenGate() {
        return lineDrive(() -> {
            cruise();
            return robotPoses().openGatePos;
        });
    }

    public AutoCommand driveOffLine() {
        return lineDrive(() -> {
            cruise();
            return robotPoses().strafePos;
        });
    }

    public AutoCommand collectFirstSpike() {
        return pathDrive(
                () -> {
                    loaded();
                    return robotPoses().collect1Pre;
                },
                () -> robotPoses().firstSpikeEnd
        );
    }

    public AutoCommand collectSecondSpike() {
        return pathDrive(
                () -> {
                    loaded();
                    return robotPoses().collect2Mid;
                },
                () -> robotPoses().collect2Pre,
                () -> robotPoses().secondSpikeEnd
        );
    }

    public AutoCommand collectThirdSpike() {
        return pathDrive(
                () -> {
                    loaded();
                    return robotPoses().collect3Pre;
                },
                () -> robotPoses().thirdSpikeEnd
        );
    }

    public AutoCommand driveToShootViaPass() {
        return pathDrive(
                () -> {
                    precision();
                    return robotPoses().collect3PrePass;
                },
                () -> robotPoses().close1Shooting
        );
    }

    public RobotPoses robotPoses() {
        return isRed() ? redPoses : bluePoses;
    }

    // ---- Named subsystem commands: connect these to fields on BrainSTEMRobot ----

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
