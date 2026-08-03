package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;

/**
 * RobotModel SysId — top speed + accel/decel.
 * <p>
 * Clear a long straight run. Left stick = drive (same as tele).
 * <ol>
 *   <li>Slam stick full forward and hold until speed plateaus → top speed + maxAccel</li>
 *   <li>Release stick (let it brake) → maxDecel</li>
 *   <li>A = reset peaks · B = stop motors</li>
 * </ol>
 * Paste the suggested values into {@code RobotConfiguration.createRobotModel()}.
 */
@TeleOp(name = "SysId Accel / Top Speed", group = "SysId")
public class SysIdAccelOpMode extends LinearOpMode {

    private static final double FULL_STICK = 0.85;
    private static final double RELEASED = 0.15;
    private static final double MIN_DT = 0.008;
    private static final double MIN_SPEED_FOR_DECEL = 8.0; // in/s

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        robot.setStartPose(new double[]{0, 0, 0});

        telemetry.addLine("SysId: Accel / Decel / Top Speed");
        telemetry.addLine("1) Slam left stick FULL forward, hold until speed flats");
        telemetry.addLine("2) RELEASE stick to measure decel");
        telemetry.addLine("A=reset peaks  B=estop");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.setStartPose(new double[]{0, 0, 0});
        robot.update();

        ElapsedTime clock = new ElapsedTime();
        double lastT = 0;
        double lastSpeed = 0;
        boolean haveSample = false;
        boolean wasFullThrottle = false;

        double peakSpeed = 0;
        double peakAccel = 0;
        double peakDecel = 0; // positive magnitude

        while (opModeIsActive()) {
            robot.update();

            if (gamepad1.a) {
                peakSpeed = 0;
                peakAccel = 0;
                peakDecel = 0;
                haveSample = false;
            }
            if (gamepad1.b) {
                robot.drive.setMotorPowers(0, 0, 0, 0);
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x * 0.75;
            driveMecanum(robot, y, x, rx);

            double stickMag = Math.hypot(x, y);
            boolean fullThrottle = stickMag >= FULL_STICK;
            boolean released = stickMag <= RELEASED;

            double vx = robot.pinpoint.getVelocity().getX();
            double vy = robot.pinpoint.getVelocity().getY();
            double speed = Math.hypot(vx, vy);

            double t = clock.seconds();
            double dt = t - lastT;
            double instantAccel = 0;
            if (haveSample && dt >= MIN_DT) {
                instantAccel = (speed - lastSpeed) / dt;
                if (fullThrottle && instantAccel > peakAccel) {
                    peakAccel = instantAccel;
                }
                // Decel only after a real run (was full, now released, still moving)
                if (released && wasFullThrottle && speed >= MIN_SPEED_FOR_DECEL && instantAccel < 0) {
                    peakDecel = Math.max(peakDecel, -instantAccel);
                }
                if (speed > peakSpeed) {
                    peakSpeed = speed;
                }
                lastT = t;
                lastSpeed = speed;
            } else if (!haveSample) {
                lastT = t;
                lastSpeed = speed;
                haveSample = true;
            }

            if (fullThrottle) {
                wasFullThrottle = true;
            }
            if (released && speed < 2.0) {
                wasFullThrottle = false;
            }

            double suggestedVOverride = peakSpeed > 1 ? peakSpeed * 0.95 : 0;
            double suggestedAccel = peakAccel > 1 ? peakAccel * 0.90 : 0;
            double suggestedDecel = peakDecel > 1 ? peakDecel * 0.90 : 0;

            telemetry.addLine("--- live ---");
            telemetry.addData("stick", "%.2f %s", stickMag, fullThrottle ? "FULL" : released ? "RELEASED" : "");
            telemetry.addData("speed in/s", "%.1f", speed);
            telemetry.addData("accel in/s^2", "%.1f", instantAccel);
            telemetry.addLine("--- peaks (paste into RobotModel) ---");
            telemetry.addData("top speed", "%.1f  → maxVelocityOverride(%.1f)", peakSpeed, suggestedVOverride);
            telemetry.addData("maxAccel", "%.1f  → maxAcceleration(%.1f)", peakAccel, suggestedAccel);
            telemetry.addData("maxDecel", "%.1f  → maxDeceleration(%.1f)", peakDecel, suggestedDecel);
            telemetry.addLine("A reset · B estop · long straight, no turn");
            telemetry.update();
        }

        robot.drive.setMotorPowers(0, 0, 0, 0);
    }

    private static void driveMecanum(BrainSTEMRobot robot, double y, double x, double rx) {
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }
        robot.drive.setMotorPowers(fl, fr, bl, br);
    }
}
