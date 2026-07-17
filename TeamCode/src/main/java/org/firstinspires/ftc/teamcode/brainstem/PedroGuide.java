package org.firstinspires.ftc.teamcode.brainstem;

import com.pedropathing.auto.PedroBrainSTEMBridge;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.ExternalPoseLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.brainstem.auto.RobotActions;

/**
 * Team-owned factory for connecting BrainSTEM hardware to Pedro.
 * Does not create or start an auton.
 */
public final class PedroGuide {
    private PedroGuide() {}

    public static Follower buildFollower(
            HardwareMap hardwareMap,
            ExternalPoseLocalizer poseFeed,
            FollowerConstants followerConstants,
            MecanumConstants mecanumConstants,
            RobotModel robotModel
    ) {
        PoseConverter.useFTCCoordinates();
        Follower follower = new Follower(
                followerConstants,
                poseFeed,
                new Mecanum(hardwareMap, mecanumConstants)
        );
        follower.setMotionModel(robotModel);
        // Caller applies team feedback tuning after the model creates the predictive follower.
        return follower;
    }

    public static PedroBrainSTEMBridge createBridge(
            Follower follower,
            ExternalPoseLocalizer poseFeed
    ) {
        PedroBrainSTEMBridge bridge = new PedroBrainSTEMBridge(follower, poseFeed);
        bridge.setAttachedToRobotLoop(true);
        return bridge;
    }

    public static PedroBrainSTEMBridge createBridge(
            HardwareMap hardwareMap,
            double startX,
            double startY,
            double startHeadingRad,
            RobotConfiguration configuration,
            RobotModel robotModel
    ) {
        PoseConverter.useFTCCoordinates();
        ExternalPoseLocalizer poseFeed = new ExternalPoseLocalizer();
        poseFeed.setStartPose(Pose.fromField(startX, startY, startHeadingRad));
        Follower follower = buildFollower(
                hardwareMap,
                poseFeed,
                configuration.createFollowerConstants(),
                configuration.createMecanumConstants(),
                robotModel
        );
        configuration.configurePredictiveFollower(follower);
        return createBridge(follower, poseFeed);
    }

    public static RobotActions createActions(BrainSTEMRobot robot) {
        return new RobotActions(robot);
    }
}
