package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test Drivetrain Motors", group = "Test")
public class DrivetrainTestTele extends LinearOpMode {

    public static double POWER = 0.3;

    private DcMotorEx fl, br, bl, fr;

    @Override
    public void runOpMode() {
//        MecanumConstants cfg = new RobotConfiguration().createMecanumConstants();

         fl = hardwareMap.get(DcMotorEx.class, "FL");
         fr = hardwareMap.get(DcMotorEx.class, "FR");
         bl = hardwareMap.get(DcMotorEx.class, "BL");
         br = hardwareMap.get(DcMotorEx.class, "BR");


        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("back right power", br.getPower());

        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
//            updateD1Dr

            setMotorPowers(0.3, 0.3, 0.3, 0.3);

            telemetry.addData("FL power", fl.getPower());
            telemetry.addData("BL power", bl.getPower());
            telemetry.addData("FR power", fr.getPower());
            telemetry.addData("BR power", br.getPower());

            telemetry.update();
        }

    }



    private void updateD1Drive() {

        // DRIVING ==========================================
        double y = -gamepad1.left_stick_y * 0.99;
        double x = gamepad1.left_stick_x * 0.99;
        double rx = gamepad1.right_stick_x * 0.75;





        setMotorPowers(
                y + x + rx,
                y - x - rx,
                y - x + rx,
                y + x - rx
        );
    }

    public void setMotorPowers(double FL, double FR, double BL, double BR) {

        fr.setPower(FR);
        fl.setPower(FL);
        br.setPower(BR);
        bl.setPower(BL);
    }
}
