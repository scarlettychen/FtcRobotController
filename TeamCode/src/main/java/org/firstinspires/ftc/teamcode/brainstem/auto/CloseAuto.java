package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.auto.AlliancePoses;
import com.pedropathing.auto.AutoCommand;
import com.pedropathing.auto.AutoMode;
import com.pedropathing.auto.FunctionalCommand;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;

/**
 * Example match auton composed only from named {@link RobotActions}.
 *
 * <p>Each auton owns its start pose by overriding {@link #getStartPose()}.
 */
public class CloseAuto extends AutoMode {
    public static final double[] BLUE_START = AlliancePoses.xyz(-65, -41.75, 0);
    public static final double[] RED_START = AlliancePoses.xyz(-65, 41.75, 0);

    private final RobotActions robot;

    public CloseAuto(BrainSTEMRobot robot, RobotActions actions) {
        super(robot.follower, actions);
        this.robot = actions;
    }

    @Override
    public double[] getStartPose() {
        return isRed() ? RED_START : BLUE_START;
    }

    @Override
    public void run() {
        sequence(
                robot.driveToCloseShoot(),

                robot.collectFirstSpike(),
                robot.driveToOpenGate(),
                robot.driveToGoal(),


                robot.collectSecondSpike(),
                robot.driveToShootViaPass(),


                robot.driveOffLine()
        );
    }

    private AutoCommand waitBrief() {
        return FunctionalCommand.waitSeconds(0.2);
    }
}
