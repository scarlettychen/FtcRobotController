package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.auto.PedroBrainSTEMBridge;
import com.pedropathing.auto.PedroDrive;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.ExternalPoseLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.RobotActions;

/**
 * Factory helpers for {@link org.firstinspires.ftc.teamcode.BrainSTEMRobot} + Pedro hardware.
 * Does not create or start an auton — OpModes own scheduling / {@code AutoMode}.
 *
 * <p><b>Motors:</b> Pedro {@link Mecanum} writes path powers during auto.
 * If TeamCode also owns a Road Runner drivetrain, do not command both at once.
 */
public final class PedroGuide {
    private PedroGuide() {}

    /** Follower that reads pose from BrainSTEM via {@link ExternalPoseLocalizer}. */
    public static Follower buildFollower(
            HardwareMap hardwareMap,
            ExternalPoseLocalizer poseFeed,
            FollowerConstants followerConstants,
            MecanumConstants mecanumConstants
    ) {
        PoseConverter.useFTCCoordinates();
        return new Follower(
                followerConstants,
                poseFeed,
                new Mecanum(hardwareMap, mecanumConstants)
        );
    }

    /** Bridge with no auton bound — OpMode constructs the auton separately. */
    public static PedroBrainSTEMBridge createBridge(
            Follower follower,
            ExternalPoseLocalizer poseFeed
    ) {
        PedroBrainSTEMBridge bridge = new PedroBrainSTEMBridge(follower, poseFeed);
        bridge.setAttachedToRobotLoop(true);
        return bridge;
    }

    /**
     * Pose feed + follower + bridge for BrainSTEMRobot.
     * Motor names default to FL / BL / FR / BR.
     */
    public static PedroBrainSTEMBridge createBridge(
            HardwareMap hardwareMap,
            double startX,
            double startY,
            double startHeadingRad
    ) {
        return createBridge(
                hardwareMap,
                startX,
                startY,
                startHeadingRad,
                new FollowerConstants(),
                defaultMecanumConstants()
        );
    }

    public static PedroBrainSTEMBridge createBridge(
            HardwareMap hardwareMap,
            double startX,
            double startY,
            double startHeadingRad,
            FollowerConstants followerConstants,
            MecanumConstants mecanumConstants
    ) {
        PoseConverter.useFTCCoordinates();
        ExternalPoseLocalizer poseFeed = new ExternalPoseLocalizer();
        poseFeed.setStartPose(Pose.fromField(startX, startY, startHeadingRad));
        Follower follower = buildFollower(hardwareMap, poseFeed, followerConstants, mecanumConstants);
        return createBridge(follower, poseFeed);
    }

    /** Hardware names matching common TeamCode MecanumDrive configs (FL/BL/FR/BR). */
    public static MecanumConstants defaultMecanumConstants() {
        return new MecanumConstants()
                .leftFrontMotorName("FL")
                .leftRearMotorName("BL")
                .rightFrontMotorName("FR")
                .rightRearMotorName("BR")
                .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    }

    public static RobotActions createActions(Follower follower) {
        return new RobotActions(new PedroDrive(follower));
    }
}
