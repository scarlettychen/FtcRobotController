package org.firstinspires.ftc.teamcode.brainstem.subsystems;

import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.brainstem.RobotConfiguration;

// raw mecanum motors from robotconfig. no pedro teleop
public class Drive {
    private final DcMotorEx FL;
    private final DcMotorEx FR;
    private final DcMotorEx BL;
    private final DcMotorEx BR;

    public Drive(HardwareMap hardwareMap, RobotConfiguration configuration) {
        MecanumConstants cfg = configuration.createMecanumConstants();

        FL = hardwareMap.get(DcMotorEx.class, cfg.leftFrontMotorName);
        FR = hardwareMap.get(DcMotorEx.class, cfg.rightFrontMotorName);
        BL = hardwareMap.get(DcMotorEx.class, cfg.leftRearMotorName);
        BR = hardwareMap.get(DcMotorEx.class, cfg.rightRearMotorName);

        FL.setDirection(cfg.leftFrontMotorDirection);
        FR.setDirection(cfg.rightFrontMotorDirection);
        BL.setDirection(cfg.leftRearMotorDirection);
        BR.setDirection(cfg.rightRearMotorDirection);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setMotorPowers(0, 0, 0, 0);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        FR.setPower(fr);
        FL.setPower(fl);
        BR.setPower(br);
        BL.setPower(bl);
    }
}
