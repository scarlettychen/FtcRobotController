package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.auto.PedroDrive;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.RoadRunnerCoordinates;

@Autonomous(name = "Pedro Drive Forward", group = "Pedro")
public class DriveForwardOpMode extends LinearOpMode {

    public static final double DISTANCE_INCHES = 48.0;

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        PedroDrive drive = new PedroDrive(robot.follower);
        drive.setExternalLoop(true);
        drive.settleEnd(false);

        telemetry.addLine("Drive forward " + DISTANCE_INCHES + " in (robot heading)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.update();
        Pose bakeStart = robot.follower.getPose();
        drive.forwardDrive(DISTANCE_INCHES);

        while (opModeIsActive() && drive.isBusy()) {
            robot.update();
            drive.update();

            Pose pedro = robot.pinpoint.getPose();
            Pose field = pedro.getAsCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
            telemetry.addData("bake start", "(%.1f, %.1f, %.0f°)",
                    bakeStart.getX(), bakeStart.getY(), Math.toDegrees(bakeStart.getHeading()));
            telemetry.addData("fieldX", "%.2f", field.getX());
            telemetry.addData("fieldY", "%.2f", field.getY());
            telemetry.addData("pedroX", "%.2f", pedro.getX());
            telemetry.addData("pedroY", "%.2f", pedro.getY());
            telemetry.addData("pathDone", "%.2f", robot.follower.getPathCompletion());
            telemetry.addData("busy", robot.follower.isBusy());
            telemetry.update();
        }

        robot.follower.breakFollowing();
        sleep(500);
    }
}
