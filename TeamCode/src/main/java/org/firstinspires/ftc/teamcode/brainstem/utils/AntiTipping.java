package org.firstinspires.ftc.teamcode.brainstem.utils;


import java.util.function.DoubleSupplier;

/**
 * FTC port of TechMaker Robotics' FRC "AntiTipping" library (Chief Delphi, Nov 2025).
 *
 * Original algorithm: a proportional correction on pitch/roll that kicks in once a
 * tipping threshold is crossed, outputting a corrective velocity you swap in for the
 * driver's command until the robot settles.
 *
 * FTC differences from the FRC version:
 *   - No ChassisSpeeds type, so this outputs a simple DriveCorrection(forward, strafe)
 *     in [-1, 1] that you feed into your existing mecanum power-mixing math instead of
 *     the driver's joystick values.
 *   - Added hysteresis (separate enter/exit thresholds) so the correction doesn't
 *     chatter on/off right at the boundary -- this was a known rough edge in the
 *     original design that came up in discussion of the FRC version.
 */
public class AntiTipping {

    public static final class DriveCorrection {
        public final double forward;
        public final double strafe;

        public DriveCorrection(double forward, double strafe) {
            this.forward = forward;
            this.strafe = strafe;
        }

        public static final DriveCorrection NONE = new DriveCorrection(0, 0);
    }

    private final DoubleSupplier pitchDegrees;
    private final DoubleSupplier rollDegrees;

    private double kP;
    private double enterThresholdDegrees;
    private double exitThresholdDegrees; // hysteresis band; must be < enterThresholdDegrees
    private double maxCorrectionPower;

    private boolean tipping = false;
    private DriveCorrection lastCorrection = DriveCorrection.NONE;

    /**
     * @param pitchDegrees        supplier for current pitch, e.g. imu::getPitch
     * @param rollDegrees         supplier for current roll, e.g. imu::getRoll
     * @param kP                  proportional gain applied to the tip angle
     * @param tippingThresholdDegrees angle (deg) at which correction engages
     * @param maxCorrectionPower  clamp on the corrective output, in motor-power units [0, 1]
     */
    public AntiTipping(DoubleSupplier pitchDegrees, DoubleSupplier rollDegrees,
                       double kP, double tippingThresholdDegrees, double maxCorrectionPower) {
        this.pitchDegrees = pitchDegrees;
        this.rollDegrees = rollDegrees;
        this.kP = kP;
        this.enterThresholdDegrees = tippingThresholdDegrees;
        this.exitThresholdDegrees = tippingThresholdDegrees * 0.7; // sensible default; tune on Panels
        this.maxCorrectionPower = maxCorrectionPower;
    }

    /** Call every loop. Returns zero correction when not tipping. */
    public DriveCorrection calculate() {
        double pitch = pitchDegrees.getAsDouble();
        double roll = rollDegrees.getAsDouble();
        double magnitude = Math.max(Math.abs(pitch), Math.abs(roll));

        if (tipping) {
            // Already correcting -- only release once we've dropped below the exit threshold.
            tipping = magnitude > exitThresholdDegrees;
        } else {
            tipping = magnitude > enterThresholdDegrees;
        }

        if (!tipping) {
            lastCorrection = DriveCorrection.NONE;
            return lastCorrection;
        }

        // Counter-drive: nose pitched down -> drive backward to bring the front wheels
        // down; rolled to one side -> strafe the other way. Sign depends on your IMU's
        // mounting orientation -- verify with the robot propped up before trusting it
        // on the ground (per TechMaker's own safety note).
        double forwardCorrection = clamp(-kP * pitch, -maxCorrectionPower, maxCorrectionPower);
        double strafeCorrection = clamp(-kP * roll, -maxCorrectionPower, maxCorrectionPower);

        lastCorrection = new DriveCorrection(forwardCorrection, strafeCorrection);
        return lastCorrection;
    }

    public boolean isTipping() {
        return tipping;
    }

    public DriveCorrection getLastCorrection() {
        return lastCorrection;
    }

    public void setTippingThreshold(double enterDegrees, double exitDegrees) {
        this.enterThresholdDegrees = enterDegrees;
        this.exitThresholdDegrees = exitDegrees;
    }

    public void setMaxCorrectionPower(double power) {
        this.maxCorrectionPower = power;
    }

    public void setKP(double kP) {
        this.kP = kP;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
