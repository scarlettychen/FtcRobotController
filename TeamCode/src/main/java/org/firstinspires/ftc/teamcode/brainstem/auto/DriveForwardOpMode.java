package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.FieldCoords;
import org.firstinspires.ftc.teamcode.brainstem.follower.FollowerOutput;
import org.firstinspires.ftc.teamcode.brainstem.follower.PathFollower;
import org.firstinspires.ftc.teamcode.brainstem.follower.PathSpec;

@Autonomous(name = "Pedro Drive Forward", group = "Pedro")
public class DriveForwardOpMode extends LinearOpMode {

    public static final double DISTANCE_INCHES = 48.0;

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        PathFollower pathFollower = robot.createPathFollower();

        telemetry.addLine("Drive forward " + DISTANCE_INCHES + " in (robot heading)");
        telemetry.addLine("PathSpec + PathFollower");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.update();
        double[] bakeStart = robot.getFieldPose();
        pathFollower.startPath(PathSpec.forward("drive-forward", bakeStart, DISTANCE_INCHES));

        while (opModeIsActive() && !pathFollower.isFinished()) {
            robot.update();
            double[] field = robot.getFieldPose();
            FollowerOutput out = pathFollower.update(field[0], field[1], field[2]);

            telemetry.addData("bake start", FieldCoords.format(bakeStart));
            telemetry.addData("field", FieldCoords.format(field));
            telemetry.addData("pathDone", "%.2f", out.pathCompletion);
            telemetry.addData("crossTrack", "%.2f", out.crossTrackError);
            telemetry.addData("finished", pathFollower.isFinished());
            telemetry.update();
        }

        pathFollower.cancel();
        sleep(500);
    }
}
