package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;

/**
 * RobotModel SysId — friction coefficient (sets {@code maxLateralAcceleration = μ·g}).
 * <p>
 * Two quick tests (use A to reset between them):
 * <ol>
 *   <li><b>Strafe</b> — slam stick full LEFT or RIGHT on carpet until it chatters/slips.
 *       Peak |lateral accel| / g → μ</li>
 *   <li><b>Coast</b> (optional check) — drive full forward, then hit Y to coast (float).
 *       Coast |decel| / g → rolling μ (usually lower than strafe)</li>
 * </ol>
 * Prefer the <b>strafe</b> number for {@code frictionCoefficient(...)} — that matches curve FF.
 */
@TeleOp(name = "SysId Friction μ", group = "SysId")
public class SysIdFrictionOpMode extends LinearOpMode {

    private static final double G = 386.09; // in/s^2
    private static final double FULL_STICK = 0.85;
    private static final double MIN_DT = 0.008;

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        robot.setStartPose(new double[]{0, 0, 0});

        telemetry.addLine("SysId: Friction coefficient μ");
        telemetry.addLine("STRAFE: full left/right until slip → best for RobotModel");
        telemetry.addLine("Y = cut power + COAST (float) after a forward run");
        telemetry.addLine("A=reset  B=brake stop");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.setStartPose(new double[]{0, 0, 0});
        robot.update();
        setBrake(robot, true);

        ElapsedTime clock = new ElapsedTime();
        double lastT = 0;
        double lastVx = 0;
        double lastVy = 0;
        boolean haveSample = false;
        boolean coasting = false;

        double peakLatAccel = 0;   // |a| sideways, robot frame
        double peakCoastDecel = 0; // |a| while coasting

        while (opModeIsActive()) {
            robot.update();

            if (gamepad1.a) {
                peakLatAccel = 0;
                peakCoastDecel = 0;
                haveSample = false;
                coasting = false;
                setBrake(robot, true);
            }
            if (gamepad1.b) {
                coasting = false;
                setBrake(robot, true);
                robot.drive.setMotorPowers(0, 0, 0, 0);
            }
            if (gamepad1.y) {
                // coast stop for rolling-friction check
                coasting = true;
                setBrake(robot, false);
                robot.drive.setMotorPowers(0, 0, 0, 0);
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x * 0.75;

            if (!coasting) {
                setBrake(robot, true);
                driveMecanum(robot, y, x, rx);
            } else {
                robot.drive.setMotorPowers(0, 0, 0, 0);
                double speed = Math.hypot(
                        robot.pinpoint.getVelocity().getX(),
                        robot.pinpoint.getVelocity().getY());
                if (speed < 2.0) {
                    coasting = false;
                    setBrake(robot, true);
                }
            }

            double heading = robot.pinpoint.getPose().getHeading();
            double fieldVx = robot.pinpoint.getVelocity().getX();
            double fieldVy = robot.pinpoint.getVelocity().getY();
            // robot-frame: forward / strafe
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double robotForward = fieldVx * cos + fieldVy * sin;
            double robotStrafe = -fieldVx * sin + fieldVy * cos;

            double t = clock.seconds();
            double dt = t - lastT;
            if (haveSample && dt >= MIN_DT) {
                double ax = (fieldVx - lastVx) / dt;
                double ay = (fieldVy - lastVy) / dt;
                // lateral accel ≈ change in robot-strafe velocity
                double lastStrafe = -lastVx * sin + lastVy * cos;
                double aLat = (robotStrafe - lastStrafe) / dt;
                double aMag = Math.hypot(ax, ay);

                boolean strafingHard = Math.abs(x) >= FULL_STICK && Math.abs(y) < 0.35;
                if (strafingHard) {
                    peakLatAccel = Math.max(peakLatAccel, Math.abs(aLat));
                }
                if (coasting && aMag > 0.5) {
                    // while slowing, accel is opposite velocity
                    double speed = Math.hypot(fieldVx, fieldVy);
                    if (speed > 4.0) {
                        peakCoastDecel = Math.max(peakCoastDecel, aMag);
                    }
                }

                lastT = t;
                lastVx = fieldVx;
                lastVy = fieldVy;
            } else if (!haveSample) {
                lastT = t;
                lastVx = fieldVx;
                lastVy = fieldVy;
                haveSample = true;
            }

            double muStrafe = peakLatAccel / G;
            double muCoast = peakCoastDecel / G;
            // slight derate for safety in model
            double suggestedMu = muStrafe > 0.05 ? muStrafe * 0.85 : 0;

            telemetry.addLine("--- live ---");
            telemetry.addData("mode", coasting ? "COAST" : "DRIVE");
            telemetry.addData("fwd / strafe in/s", "%.1f / %.1f", robotForward, robotStrafe);
            telemetry.addData("stick x (strafe)", "%.2f", x);
            telemetry.addLine("--- peaks ---");
            telemetry.addData("peak |a_lat| in/s^2", "%.1f", peakLatAccel);
            telemetry.addData("μ strafe (a/g)", "%.3f  → frictionCoefficient(%.3f)", muStrafe, suggestedMu);
            telemetry.addData("peak coast |a|", "%.1f  (μ≈%.3f)", peakCoastDecel, muCoast);
            telemetry.addData("→ maxLatAccel", "%.1f in/s^2", suggestedMu * G);
            telemetry.addLine("A reset · Y coast · B brake · prefer strafe μ");
            telemetry.update();
        }

        setBrake(robot, true);
        robot.drive.setMotorPowers(0, 0, 0, 0);
    }

    private static void setBrake(BrainSTEMRobot robot, boolean brake) {
        // Drive motors aren't exposed; coast via zero power + BRAKE is default.
        // For true coast we need FLOAT — remap briefly through hardwareMap names from config.
        DcMotor.ZeroPowerBehavior behavior =
                brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        try {
            robot.hardwareMap.get(DcMotor.class, "FL").setZeroPowerBehavior(behavior);
            robot.hardwareMap.get(DcMotor.class, "FR").setZeroPowerBehavior(behavior);
            robot.hardwareMap.get(DcMotor.class, "BL").setZeroPowerBehavior(behavior);
            robot.hardwareMap.get(DcMotor.class, "BR").setZeroPowerBehavior(behavior);
        } catch (Exception ignored) {
            // names always FL/FR/BL/BR in RobotConfiguration
        }
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
