package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.auto.AutoCommand;
import com.pedropathing.auto.AutoMode;
import com.pedropathing.auto.FunctionalCommand;
import com.pedropathing.auto.PedroDrive;
import com.pedropathing.follower.Follower;

/**
 * Reference close auto — one routine works for Blue or Red via {@link #setAlliance(boolean)}.
 *
 * <pre>{@code
 * BrainSTEMRobot robot = new BrainSTEMRobot(hw, telemetry, this, sx, sy, sh);
 * RobotActions actions = PedroGuide.createActions(robot.follower);
 * actions.setAlliance(isRed);
 * CloseAuto auto = new CloseAuto(robot.follower, actions);
 * auto.setExternalLoop(true);
 * waitForStart();
 * auto.start();
 * while (opModeIsActive() && !auto.isFinished()) {
 *     robot.syncPose(...);
 *     robot.update();
 *     auto.update();
 * }
 * auto.stop();
 * }</pre>
 */
public class CloseAuto extends AutoMode {
    private final RobotActions bot;

    public CloseAuto(Follower follower, RobotActions bot) {
        super(follower, bot);
        this.bot = bot;
    }

    public CloseAuto(Follower follower) {
        this(follower, new RobotActions(new PedroDrive(follower)));
    }

    public CloseAuto(Follower follower, boolean red) {
        this(follower);
        setAlliance(red);
    }

    @Override
    public void run() {
        startPoseFromAlliance();

        sequence(
                parallel(
                        bot.shooterTurnOnClose(),
                        bot.driveToCloseShoot()
                ),
                bot.rampUp(),
                waitBrief(),
                bot.moveSpindexer360(),
                bot.rampDown(),
                bot.shooterIdle(),

                parallel(
                        bot.setCollectorOn(),
                        bot.collectFirstSpike()
                ),

                parallel(
                        bot.driveToOpenGate(),
                        bot.shooterTurnOnClose()
                ),

                parallel(
                        bot.driveToGoal(),
                        bot.setCollectorOff()
                ),

                bot.rampUp(),
                waitBrief(),
                bot.moveSpindexer360(),
                bot.rampDown(),
                bot.shooterIdle(),

                parallel(
                        bot.setCollectorOn(),
                        bot.collectSecondSpike()
                ),

                parallel(
                        bot.shooterTurnOnClose(),
                        bot.driveToShootViaPass()
                ),

                bot.setCollectorOff(),
                bot.rampUp(),
                waitBrief(),
                bot.moveSpindexer360(),
                bot.rampDown(),
                bot.shooterIdle(),

                bot.driveOffLine()
        );
    }

    private AutoCommand waitBrief() {
        return FunctionalCommand.waitSeconds(0.2);
    }
}
