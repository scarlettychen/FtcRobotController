package org.firstinspires.ftc.teamcode.brainstem;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.trajectory.PredictiveTrajectoryFollower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Team-editable Pedro hardware and motion configuration.
 *
 * <p>This is the first file to edit for motor names/directions and physical motion tuning.
 */
public class RobotConfiguration {

    public FollowerConstants createFollowerConstants() {
        return new FollowerConstants();
    }

    public MecanumConstants createMecanumConstants() {
        // Directions match this robot's wiring (verified with forwardDrive).
        // If "forward" becomes a strafe after a motor swap, restore Pedro-style
        // left=REVERSE / right=FORWARD and re-check.
        return new MecanumConstants()
                .leftFrontMotorName("frontLeftMotor")
                .leftRearMotorName("backLeftMotor")
                .rightFrontMotorName("frontRightMotor")
                .rightRearMotorName("backRightMotor")
                .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * goBILDA Pinpoint settings. Hardware map name must match the RC config device name.
     * Offsets match the Decode robot (mm from center).
     */
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
                .feedforward(0.05, 0.012, 0.002);
    }

    /** Team-editable feedback gains layered on top of RobotModel feedforward. */
    public void configurePredictiveFollower(Follower follower) {
        PredictiveTrajectoryFollower predictive = follower.getTrajectoryFollower();
        // Slightly stronger cross-track so Bezier bulge is tracked, not only chord-cut.
        predictive.setGains(0.1, 0.015, 0.14, 0.8, 0.05);
        predictive.kVOmega = 0.12;
        predictive.kAAlpha = 0.02;
    }
}
