package org.firstinspires.ftc.teamcode.brainstem;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.ExternalPoseLocalizer;
import com.pedropathing.util.Component;

// pedro half of the loop — pose feed + idle follower ticks
// opmodes own scheduling, this never starts an auton
public class PedroBrainSTEMBridge implements Component {
    private final Follower follower;
    private final ExternalPoseLocalizer localizer;
    private boolean attachedToRobotLoop = true;

    public PedroBrainSTEMBridge(Follower follower, ExternalPoseLocalizer localizer) {
        this.follower = follower;
        this.localizer = localizer;
    }

    public Follower getFollower() {
        return follower;
    }

    public ExternalPoseLocalizer getPoseFeed() {
        return localizer;
    }

    public void setAttachedToRobotLoop(boolean attached) {
        this.attachedToRobotLoop = attached;
    }

    public boolean isAttachedToRobotLoop() {
        return attachedToRobotLoop;
    }

    public void syncPoseFromRobot(double x, double y, double headingRad,
                                  double vx, double vy, double omega) {
        localizer.setState(x, y, headingRad, vx, vy, omega);
    }

    public void syncPoseFromRobot(double x, double y, double headingRad,
                                  double vx, double vy, double omega,
                                  double localizationConfidence) {
        syncPoseFromRobot(x, y, headingRad, vx, vy, omega);
        if (follower.getMotionModel() != null) {
            follower.getMotionModel().setLocalizationConfidence(localizationConfidence);
        }
    }

    public void syncPoseFromRobot(double[] posVel) {
        if (posVel == null || posVel.length < 6) return;
        syncPoseFromRobot(posVel[0], posVel[1], posVel[2], posVel[3], posVel[4], posVel[5]);
    }

    @Override
    public void reset() {
        follower.breakFollowing();
    }

    @Override
    public void update() {
        if (!attachedToRobotLoop && follower != null) {
            follower.update();
        }
    }

    @Override
    public String test() {
        return "PedroBrainSTEMBridge";
    }
}
