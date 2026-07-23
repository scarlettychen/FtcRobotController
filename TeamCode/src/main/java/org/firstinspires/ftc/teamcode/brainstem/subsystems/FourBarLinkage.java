package org.firstinspires.ftc.teamcode.brainstem.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Component;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.brainstem.utils.HardwareNames;
import org.firstinspires.ftc.teamcode.brainstem.utils.PIDController;


@Configurable
public class FourBarLinkage implements Component {

//    public static boolean leftMotorOn = false;

    public static double TESTING_POWER = 0.2;
    public static boolean rightMotorOn = true;

    private double targetPosition = 0.0;

    public static int HIGH_POS = 479;
    public static int LOW_POS = 177;
    public static int DOWN_POS = 40;

    public static int ERROR_THRESHOLD = 20;

    public static double PARACHUTE_POWER = 0.05;



    public static double kP = 0.005, kI = 0.0, kD = 0.00000,
            kG = 0.2;




    private final DcMotorEx right;
    private final PIDController pid;

    private final Telemetry telemetry;

    public double batteryVoltage = 13.0;

    private double error = 0.0;
    public double direction;

    public enum LinkState {
        DOWN,
        SCORE_LOW,
        SCORE_HIGH,
        TESTING,
        OFF
    }

    public LinkState state;



    public LinkState getState() {
        return state;
    }

    public FourBarLinkage(HardwareMap hardwareMap, Telemetry tel) {

        this.telemetry = tel;

//        left = hardwareMap.get(DcMotorEx.class, HardwareNames.liftLeft);
        right = hardwareMap.get(DcMotorEx.class, HardwareNames.liftRight);

        // right.setDirection(DcMotor.Direction.REVERSE);

//        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        pid = new PIDController(kP, kI, kD);

        state = LinkState.DOWN; // todo: change this
    }

    @Override
    public void reset() {

    }



    @Override
    public void update() {


        telemetry.addData("Lift motor power", right.getPower());

        pid.setGains(kP, kI, kD);

        int current = right.getCurrentPosition();
        error = targetPosition - current;

        switch (state) {
            case SCORE_LOW:
                pid.setGains(kP, kI, kD);

                targetPosition = LOW_POS;
                goToTarget(current, false);

                break;
            case SCORE_HIGH:
                pid.setGains(kP, kI, kD);

                targetPosition = HIGH_POS;
                goToTarget(current, false);

                break;

            case DOWN:
                if (current > DOWN_POS + ERROR_THRESHOLD) {
                    setPower(PARACHUTE_POWER);
                } else {

                    setPower(0);
                }

                break;
            case TESTING:
//
                break;
            case OFF:
                right.setPower(0);
                break;
        }

    }

    public void setState(LinkState newState) {

        if (this.state != newState) {
            this.state = newState;

            switch (newState) {
                case SCORE_LOW:
                    pid.setGains(kP, kI, kD);
                    pid.reset();
                    targetPosition = LOW_POS;
                    break;
                case SCORE_HIGH:
                    pid.setGains(kP, kI, kD);
                    pid.reset();
                    targetPosition = HIGH_POS;
                    break;
                case DOWN:
                    targetPosition = DOWN_POS;
                    // No need to reset PID if we are bypassing it anyway
                    break;
                case OFF:
                    right.setPower(0);
                    break;
            }
        }
    }

    @Override
    public String test() {
        return "";
    }

    public void goToTarget(int current, boolean down) {

        double fb = pid.calculate(current, targetPosition);

        double kS = Math.signum(error) == 1 ? 0.15 : 0.10;

        if (down) kS = 0;
        double gravityFF = down ? 0 : kG;

        double ff = gravityFF + kS;

        double u = fb + ff;

        setPower(u);

    }

    private void setPower(double pow) {

        right.setPower(pow);
//        left.setPower(pow);
    }

    public void setBatteryVoltage(double voltage) {
        this.batteryVoltage = voltage;
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
        // they move together so one encoder is enough
        return right.getCurrentPosition();
    }


    public int getRightPosition() {
        return right.getCurrentPosition();
    }
}
