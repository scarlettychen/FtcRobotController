package org.firstinspires.ftc.teamcode.brainstem.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficientSupplier;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.util.Component;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.brainstem.utils.HardwareNames;


@Configurable
public class Intake implements Component {
    private final Telemetry telemetry;
    private final DcMotorEx motor;


    public enum IntakeState {
        OFF,
        IN,
        OUT
    }

    private double power, prevPower;

    public static double inPower = 0.99;
    public static double outPower = -0.99;

    private IntakeState intakeState;

    public Intake(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.motor = hwMap.get(DcMotorEx.class, HardwareNames.intakeName);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeState = IntakeState.OFF;
    }

    @Override
    public void reset() {

    }


    @Override
    public void update() {

        switch (intakeState) {
            case OFF:
                power = 0;
                break;
            case IN:
                power = inPower;
                break;
            case OUT:
                power = outPower;
                break;
        }

        if (prevPower != power) {
            motor.setPower(power);
        }

        prevPower = power;

    }

    @Override
    public String test() {
        return "";
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public void setIntakeState(IntakeState state) {
        this.intakeState = state;
    }
}