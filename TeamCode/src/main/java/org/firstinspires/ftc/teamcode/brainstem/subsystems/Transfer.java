package org.firstinspires.ftc.teamcode.brainstem.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Component;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.brainstem.utils.HardwareNames;

@Configurable
public class Transfer implements Component {
    private final Telemetry telemetry;
    private final DcMotorEx motor;

    public enum TransferState {
        OFF,
        IN,
        OUT
    }

    private double power, prevPower;

    public static double inPower = 0.7;
    public static double outPower = -0.7;

    private TransferState transferState;

    public Transfer(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.motor = hwMap.get(DcMotorEx.class, HardwareNames.beltName);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        transferState = TransferState.OFF;
    }

    @Override
    public void reset() {

    }

    @Override
    public void update() {

        telemetry.addData("power", power);
        telemetry.addData("act mtor power", motor.getPower());
        switch (transferState) {
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

    public TransferState getTransferState() {
        return transferState;
    }

    public void setTransferState(TransferState transferState) {
        this.transferState = transferState;
    }
}
