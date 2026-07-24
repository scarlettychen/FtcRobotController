package org.firstinspires.ftc.teamcode.brainstem.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Component;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.brainstem.utils.HardwareNames;
import org.firstinspires.ftc.teamcode.brainstem.utils.PIDController;

@Configurable
public class FourBarLinkage implements Component {

    public static double TESTING_POWER = 0.3;

    // flip if pid says "up" but the arm goes down
    public static int MOTOR_SIGN = 1;

    public static int HIGH_POS = 510;
    public static int LOW_POS = 177;
    public static int DOWN_POS = 40;

    public static int ERROR_THRESHOLD = 20;

    public static double PARACHUTE_POWER = -0.005;

    public static double kP = 0.007, kI = 0.0, kD = 0.0005;
    public static double kG = 0.2;
    public static double kS = 0.1;

    private final DcMotorEx right;
    private final PIDController pid;
    private final Telemetry telemetry;

    private double targetPosition = 0.0;
    private double lastPower = 0.0;

    public enum LinkState {
        DOWN,
        SCORE_LOW,
        SCORE_HIGH,
        TESTING,
        OFF
    }

    public LinkState state = LinkState.DOWN;

    public FourBarLinkage(HardwareMap hardwareMap, Telemetry tel) {
        this.telemetry = tel;

        right = hardwareMap.get(DcMotorEx.class, HardwareNames.liftRight);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMotorEnable();

        pid = new PIDController(kP, kI, kD);
        pid.setOutputLimit(1.0);

        state = LinkState.DOWN;
        targetPosition = DOWN_POS;
    }

    public LinkState getState() {
        return state;
    }

    public void setState(LinkState newState) {
        boolean changed = this.state != newState;
        this.state = newState;

        switch (newState) {
            case SCORE_LOW:
                targetPosition = LOW_POS;
                if (changed) {
                    pid.reset();
                }
                break;
            case SCORE_HIGH:
                targetPosition = HIGH_POS;
                if (changed) {
                    pid.reset();
                }
                break;
            case DOWN:
                targetPosition = DOWN_POS;
                break;
            case OFF:
            case TESTING:
                setPower(0);
                break;
        }
    }

    @Override
    public void reset() {}

    @Override
    public void update() {
        pid.setGains(kP, kI, kD);

        int current = right.getCurrentPosition();

        switch (state) {
            case SCORE_LOW:
                targetPosition = LOW_POS;
                goToTarget(current);
                break;
            case SCORE_HIGH:
                targetPosition = HIGH_POS;
                goToTarget(current);
                break;
            case DOWN:

                if (current > DOWN_POS + ERROR_THRESHOLD) {
                    setPower(PARACHUTE_POWER);
                } else {
                    setPower(0);
                }
                break;
            case TESTING:
                setPower(TESTING_POWER);
                break;
            case OFF:
                setPower(0);
                break;
        }

        telemetry.addData("lift state", state);
        telemetry.addData("lift pos/tgt", "%d / %.0f", current, targetPosition);
        telemetry.addData("lift power", lastPower);
        telemetry.addData("lift atTarget", atTarget());
    }

    @Override
    public String test() {
        return "";
    }

    public void goToTarget(int current) {
        double fb = pid.calculate(current, targetPosition);
        double err = targetPosition - current;
        // kG holds against gravity toward higher encoder; kS helps along motion
        double ff = kG + kS * Math.signum(err);
        setPower(fb + ff);
    }

    private void setPower(double pow) {
        lastPower = Range.clip(MOTOR_SIGN * pow, -1.0, 1.0);
        right.setPower(lastPower);
    }

    public boolean atTarget() {
        return Math.abs(desiredTarget() - getPosition()) < ERROR_THRESHOLD;
    }

    private double desiredTarget() {
        switch (state) {
            case SCORE_HIGH:
                return HIGH_POS;
            case SCORE_LOW:
                return LOW_POS;
            case DOWN:
                return DOWN_POS;
            default:
                return targetPosition;
        }
    }

    public int getPosition() {
        return right.getCurrentPosition();
    }

    public int getRightPosition() {
        return right.getCurrentPosition();
    }

    public double getLastPower() {
        return lastPower;
    }
}
