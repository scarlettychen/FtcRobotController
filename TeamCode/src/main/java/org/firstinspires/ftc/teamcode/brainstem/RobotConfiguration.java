package org.firstinspires.ftc.teamcode.brainstem;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.trajectory.PredictiveTrajectoryFollower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// pedro hardware + motion knobs. edit this first for motor names / directions
public class RobotConfiguration {

    public FollowerConstants createFollowerConstants() {
        return new FollowerConstants();
    }

    public MecanumConstants createMecanumConstants() {
        // directions match how we wired this bot
        return new MecanumConstants()
                .leftFrontMotorName("FL")
                .leftRearMotorName("BL")
                .rightFrontMotorName("FR")
                .rightRearMotorName("BR")
                .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
    }

    // pinpoint settings. hwmap name gotta match the rc config
    // offsets are mm from center (decode bot)
    public PinpointConstants createPinpointConstants() {
        return new PinpointConstants()
                .hardwareMapName("odo")
                .distanceUnit(DistanceUnit.MM)
                .forwardPodY(-28.5)
                .strafePodX(-266.7)
                .customEncoderResolution(19.894)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    public RobotModel createRobotModel() {
        return new RobotModel()
                .mass(12.0)
                .wheelRadius(1.8898)
                .motorFreeSpeed(32.67)
                .gearRatio(1.0)
                .frictionCoefficient(0.7)
                .maxAcceleration(80.0)
                .maxDeceleration(100.0)
                .maxAngularVelocity(6.0)
                .maxAngularAcceleration(20.0)
                .feedforward(0.05, 0.18, 0.003);
    }

    public void configurePredictiveFollower(Follower follower) {
        PredictiveTrajectoryFollower predictive = follower.getTrajectoryFollower();
        predictive.setGains(0.15, 0.015, 0.0, 0.8, 0.05);
        predictive.kVOmega = 0.12;
        predictive.kAAlpha = 0.02;
        predictive.endCrossTrackBoost = 1.75;
        predictive.crossMinPower = 0.0;
        predictive.closestAheadInches = 3.0;
        predictive.settleHoldSeconds = 0.2;
    }
}
