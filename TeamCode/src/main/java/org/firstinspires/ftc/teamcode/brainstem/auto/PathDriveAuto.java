package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.auto.AlliancePoses;
import com.pedropathing.auto.AutoMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;

/**
 * Straight-segment {@code pathDrive} through a few field points.
 *
 * <pre>
 * START (0, 0, 0°)
 *   → A (24, 0, 0°)     forward
 *   → B (24, 16, 90°)   left + turn
 *   → C (40, 16, 90°)   continue
 * </pre>
 */
public class PathDriveAuto extends AutoMode {
    public static final double[] START = AlliancePoses.xyz(0, 0, 0);
    public static final double[] A = AlliancePoses.xyz(-48, 0, 0);
    public static final double[] B = AlliancePoses.xyz(-48, -48, 90);
    public static final double[] C = AlliancePoses.xyz(0, -48, 90);

    private final RobotActions bot;

    public PathDriveAuto(BrainSTEMRobot robot, RobotActions actions) {
        super(robot.follower, actions);
        this.bot = actions;
    }

    @Override
    public double[] getStartPose() {
        return START;
    }

    @Override
    public void run() {
        run(sequence(
                bot.pathDriveFewPoints(A, B, C)
        ));
    }
}
