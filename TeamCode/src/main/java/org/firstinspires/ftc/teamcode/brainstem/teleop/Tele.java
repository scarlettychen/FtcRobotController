package org.firstinspires.ftc.teamcode.brainstem.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.auto.OpmodeCommands;
import org.firstinspires.ftc.teamcode.brainstem.utils.GamepadTracker;

// teleop: sticks = raw drive, buttons = opmodecommands (no pedro tele / auto-drive)
// a = resetandcollect  b = resetfeeder
// x = raise+score low  y = raise+score high
// rb = intake+transfer  lb = cancel
@Configurable
public abstract class Tele extends LinearOpMode {

    private final boolean red;

    protected BrainSTEMRobot robot;
    protected GamepadTracker gp1;
    protected GamepadTracker gp2;

    private Command active;

    protected Tele(boolean red) {
        this.red = red;
    }

    protected void run(Command command) {
        cancelCommand();
        if (command == null) {
            return;
        }
        active = command;
        Scheduler.schedule(command);
    }

    protected void cancelCommand() {
        if (active != null) {
            Scheduler.cancel(active);
            active = null;
        }
        Scheduler.reset();
    }

    @Override
    public void runOpMode() {
        robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        robot.setAlliance(red);

        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);

        telemetry.addLine("Tele");
        telemetry.addData("alliance", red ? "RED" : "BLUE");
        telemetry.addLine("A collect | B reset | X/Y raise-score | RB intake | LB cancel");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.update();
            gp1.update();
            gp2.update();

            if (gp1.isFirstA()) {
                run(OpmodeCommands.resetAndCollect(
                        robot.intake, robot.transfer, robot.lift, robot.blocker));
            }
            if (gp1.isFirstB()) {
                run(OpmodeCommands.resetAll(
                        robot.intake, robot.transfer, robot.lift, robot.blocker));
            }
            if (gp1.isFirstX()) {
                run(OpmodeCommands.raiseAndScoreLow(
                        robot.intake, robot.transfer, robot.lift, robot.blocker));
            }
            if (gp1.isFirstY()) {
                run(OpmodeCommands.raiseAndScoreHigh(
                        robot.intake, robot.transfer, robot.lift, robot.blocker));
            }
            if (gp1.isFirstRightBumper()) {
                run(OpmodeCommands.turnOnIntakeAndTransfer(robot.intake, robot.transfer));
            }
            if (gp1.isFirstLeftBumper()) {
                cancelCommand();
            }

            Scheduler.execute();

            double y = -gamepad1.left_stick_y * 0.99;
            double x = gamepad1.left_stick_x * 0.99;
            double rx = gamepad1.right_stick_x * 0.75;

            double fl = y + x + rx;
            double fr = y - x - rx;
            double bl = y - x + rx;
            double br = y + x - rx;

            double max = Math.max(
                    Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            robot.drive.setMotorPowers(fl, fr, bl, br);
            Pose field = robot.pinpoint.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
            telemetry.addData("alliance", red ? "RED" : "BLUE");
            telemetry.addData("cmd", active != null && Scheduler.isScheduled(active) ? "RUNNING" : "off");
            telemetry.addData("intake", robot.intake.getIntakeState());
            telemetry.addData("transfer", robot.transfer.getTransferState());
            telemetry.addData("lift", "%s pos=%d atTarget=%s",
                    robot.lift.getState(), robot.lift.getPosition(), robot.lift.atTarget());
            telemetry.addData("blocker", robot.blocker.getState());
            telemetry.addData("field", "(%.1f, %.1f, %.0f°)",
                    field.getX(), field.getY(), Math.toDegrees(field.getHeading()));
            telemetry.update();
        }
        Scheduler.reset();
        cancelCommand();
        robot.drive.setMotorPowers(0, 0, 0, 0);
    }
}
