package org.firstinspires.ftc.teamcode.brainstem;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.ExternalPoseLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

// wires up pedro follower + pose bridge for brainstemrobot
public final class PedroGuide {
    private PedroGuide() {}

    public static PedroBrainSTEMBridge createBridge(
            HardwareMap hardwareMap,
            RobotConfiguration configuration,
            RobotModel robotModel
    ) {
        // team field: 0° = +Y (into field), CCW+ (RoadRunnerCoordinates)
        PoseConverter.setFieldCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
        ExternalPoseLocalizer poseFeed = new ExternalPoseLocalizer();
        poseFeed.setStartPose(Pose.fromField(0, 0, 0));
        Follower follower = new Follower(
                configuration.createFollowerConstants(),
                poseFeed,
                new Mecanum(hardwareMap, configuration.createMecanumConstants())
        );
        follower.setMotionModel(robotModel);
        follower.constants.mass = robotModel.mass;
        PedroBrainSTEMBridge bridge = new PedroBrainSTEMBridge(follower, poseFeed);
        bridge.setAttachedToRobotLoop(true);
        return bridge;
    }
}
