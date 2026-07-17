package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;

@Autonomous(name = "Pedro Close Auto", group = "Pedro")
public class CloseAutoOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        RobotActions actions = new RobotActions(robot);
        CloseAuto auto = new CloseAuto(robot, actions);
        auto.setExternalLoop(true);

        boolean red = false;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b || gamepad2.b) {
                red = true;
            }
            if (gamepad1.x || gamepad2.x) {
                red = false;
            }

            auto.setAlliance(red);
            robot.setAlliance(red);

            telemetry.addLine("Pedro Close Auto");
            telemetry.addData("alliance", red ? "RED (B)" : "BLUE (X)");
            telemetry.addLine("Press B = Red, X = Blue");
            telemetry.update();
        }

        if (isStopRequested()) return;

        auto.setAlliance(red);
        robot.setAlliance(red);
        robot.setStartPose(auto.getStartPose());

        auto.start();
        while (opModeIsActive() && !auto.isFinished()) {
            robot.update();
            auto.update();

            Pose pedro = robot.pinpoint.getPose();
            telemetry.addData("alliance", red ? "RED" : "BLUE");
            telemetry.addData("busy", robot.follower.isBusy());
            telemetry.addData("pathDone", "%.2f", robot.follower.getPathCompletion());
            telemetry.addData("pedro", "(%.1f, %.1f, %.0f°)",
                    pedro.getX(), pedro.getY(), Math.toDegrees(pedro.getHeading()));
            telemetry.addData("finished", auto.isFinished());
            if (robot.follower.drivetrain instanceof com.pedropathing.ftc.drivetrains.Mecanum) {
                com.pedropathing.ftc.drivetrains.Mecanum mech =
                        (com.pedropathing.ftc.drivetrains.Mecanum) robot.follower.drivetrain;
                telemetry.addData("LF", "%.2f", mech.getMotors().get(0).getPower());
            }
            telemetry.update();
        }

        auto.stop();
        robot.follower.breakFollowing();
    }
}
