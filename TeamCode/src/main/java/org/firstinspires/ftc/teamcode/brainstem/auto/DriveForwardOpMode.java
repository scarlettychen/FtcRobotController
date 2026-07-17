package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.auto.AutoCommand;
import com.pedropathing.auto.AutoScheduler;
import com.pedropathing.auto.PedroDrive;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;

/**
 * Drive forward along the robot's current heading with live pose telemetry.
 *
 * <p>Uses {@link PedroDrive#forwardDrive(double)} so the path matches teleop "forward"
 * (Pedro X / field Y when Pinpoint heading is ~0°), not FTC field +X (Pedro Y / 90° heading).
 */
@Autonomous(name = "Pedro Drive Forward", group = "Pedro")
public class DriveForwardOpMode extends LinearOpMode {

    public static final double DISTANCE_INCHES = 24.0;

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        PedroDrive drive = new PedroDrive(robot.follower);
        drive.setExternalLoop(true);

        AutoScheduler scheduler = new AutoScheduler();
        AutoCommand move = drive.forwardDrive(DISTANCE_INCHES);

        telemetry.addLine("Drive forward " + DISTANCE_INCHES + " in (robot heading)");
        telemetry.addLine("Expect pedroX / fieldY to climb when heading ~0°");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.update();
        Pose bakeStart = robot.follower.getPose();

        scheduler.schedule(move);
        while (opModeIsActive() && scheduler.isRunning()) {
            robot.update();
            scheduler.run();

            Pose pedro = robot.pinpoint.getPose();
            Pose field = pedro.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
            Pose velocity = robot.pinpoint.getVelocity();

            telemetry.addData("bake start", "(%.1f, %.1f, %.0f°)",
                    bakeStart.getX(), bakeStart.getY(), Math.toDegrees(bakeStart.getHeading()));
            telemetry.addData("fieldX", "%.2f", field.getX());
            telemetry.addData("fieldY", "%.2f", field.getY());
            telemetry.addData("fieldH deg", "%.1f", Math.toDegrees(field.getHeading()));
            telemetry.addData("pedroX", "%.2f", pedro.getX());
            telemetry.addData("pedroY", "%.2f", pedro.getY());
            telemetry.addData("pedroH deg", "%.1f", Math.toDegrees(pedro.getHeading()));
            telemetry.addData("vx", "%.2f", velocity.getX());
            telemetry.addData("vy", "%.2f", velocity.getY());
            telemetry.addData("pathDone", "%.2f", robot.follower.getPathCompletion());
            telemetry.addData("busy", robot.follower.isBusy());
            telemetry.addData("followingTO", robot.follower.isFollowingTrajectory());
            if (robot.follower.drivetrain instanceof com.pedropathing.ftc.drivetrains.Mecanum) {
                com.pedropathing.ftc.drivetrains.Mecanum mech =
                        (com.pedropathing.ftc.drivetrains.Mecanum) robot.follower.drivetrain;
                telemetry.addData("LF pwr", "%.2f", mech.getMotors().get(0).getPower());
                telemetry.addData("LR pwr", "%.2f", mech.getMotors().get(1).getPower());
                telemetry.addData("RF pwr", "%.2f", mech.getMotors().get(2).getPower());
                telemetry.addData("RR pwr", "%.2f", mech.getMotors().get(3).getPower());
            }
            telemetry.update();
        }

        scheduler.cancel();
        robot.follower.breakFollowing();

        Pose field = robot.pinpoint.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        Pose pedro = robot.pinpoint.getPose();
        telemetry.addLine("Done");
        telemetry.addData("final fieldX", "%.2f", field.getX());
        telemetry.addData("final fieldY", "%.2f", field.getY());
        telemetry.addData("final pedroX", "%.2f", pedro.getX());
        telemetry.addData("final pedroY", "%.2f", pedro.getY());
        telemetry.update();
        sleep(1500);
    }
}
