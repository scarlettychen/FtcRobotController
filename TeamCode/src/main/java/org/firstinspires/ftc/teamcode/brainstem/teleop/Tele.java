package org.firstinspires.ftc.teamcode.brainstem.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.auto.PedroDrive;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.RoadRunnerCoordinates;
import org.firstinspires.ftc.teamcode.brainstem.auto.OpmodeCommands;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.FourBarLinkage;
import org.firstinspires.ftc.teamcode.brainstem.utils.GamepadTracker;

// teleop: sticks = raw drive, buttons = opmodecommands
// Y = limelight smart collect (Pedro) | X = cancel
// RT hold = intake+transfer in | LT hold = extake both | release = off
// B = reset | LB = open blocker | dpad / RB = raise+score
@Configurable
public abstract class Tele extends LinearOpMode {

    public static int SMART_COLLECT_TARGET = 5;
    public static double SMART_COLLECT_TIMEOUT_S = 15.0;
    public static double[] START = {-24, -63, 0};
    public static double TRIGGER_DEADBAND = 0.3;

    private boolean raised = false;

    private enum TriggerFlow { OFF, IN, OUT }

    private double time = 0;

    private final boolean red;

    protected BrainSTEMRobot robot;
    protected PedroDrive pedroDrive;
    protected GamepadTracker gp1;
    protected GamepadTracker gp2;

    private Command active;
    private boolean suppressStickDrive;
    /** Last trigger-driven flow we scheduled; null = unknown (re-apply when idle). */
    private TriggerFlow lastTriggerFlow;

    protected Tele(boolean red) {
        this.red = red;
    }

    protected void run(Command command) {
        run(command, false);
    }

    protected void run(Command command, boolean takesDrive) {
        cancelCommand();
        if (command == null) {
            return;
        }
        active = command;
        suppressStickDrive = takesDrive;
        Scheduler.schedule(command);
    }

    protected void cancelCommand() {
        if (active != null) {
            Scheduler.cancel(active);
            active = null;
        }
        suppressStickDrive = false;
        Scheduler.reset();
    }

    private boolean commandRunning() {
        return active != null && Scheduler.isScheduled(active);
    }

    @Override
    public void runOpMode() {
        robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        robot.setAlliance(red);
        robot.setStartPose(START);

        pedroDrive = new PedroDrive(robot.follower);
        pedroDrive.setExternalLoop(true);

        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);

        telemetry.addLine("Tele");
        telemetry.addData("alliance", red ? "RED" : "BLUE");
        telemetry.addData("start", "(%.0f, %.0f, %.0f°)", START[0], START[1], START[2]);
        telemetry.addLine("Y smart collect | X cancel | RT intake | LT extake | RB score | LB blocker");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.setStartPose(START);
        lastTriggerFlow = null;

        while (opModeIsActive()) {
            robot.update();
            gp1.update();
            gp2.update();


            if (gp1.isFirstRightBumper()) {
                lastTriggerFlow = null;
                if (robot.lift.getState() == FourBarLinkage.LinkState.SCORE_HIGH) {
                    run(OpmodeCommands.resetAll(
                            robot.intake, robot.transfer, robot.lift, robot.blocker));
                } else {
                    run(OpmodeCommands.raiseAndScoreHigh(
                            robot.intake, robot.transfer, robot.lift, robot.blocker));
                }
            }

            // Pedro teleop chase — must suppress raw Drive sticks (takesDrive=true)
            if (gp1.isFirstY()) {
                lastTriggerFlow = null;
                run(OpmodeCommands.smartCollect(
                        pedroDrive,
                        robot.limelight,
                        robot.intake,
                        robot.transfer,
                        robot.intakeGate,
                        SMART_COLLECT_TARGET,
                        SMART_COLLECT_TIMEOUT_S), true);
            }

            if (gp1.isFirstX()) {
                lastTriggerFlow = null;
                cancelCommand();
                robot.follower.breakFollowing();
                robot.drive.setMotorPowers(0, 0, 0, 0);
            }

            if (gp1.isFirstB() || gamepad2.bWasPressed()) {
                lastTriggerFlow = null;
                run(OpmodeCommands.resetAll(
                        robot.intake, robot.transfer, robot.lift, robot.blocker));
            }


            if (gp1.isFirstLeftBumper()) {
                lastTriggerFlow = null;
                run(OpmodeCommands.turnOnTransferAndOpenBlocker(
                        robot.transfer, robot.blocker, robot.intake));
            }

            if (gp1.isFirstDpadDown()) {
                lastTriggerFlow = null;
                run(OpmodeCommands.raiseAndScoreLow(
                        robot.intake, robot.transfer, robot.lift, robot.blocker));
            }
            if (gp1.isFirstDpadUp()) {
                lastTriggerFlow = null;
                run(OpmodeCommands.raiseAndScoreHigh(
                        robot.intake, robot.transfer, robot.lift, robot.blocker));
            }

            // held triggers → Ivy instants (edge only). skip while another cmd owns subsystems.
            // Never auto-OFF from unknown/null — that wiped transfer after LB score Instant.
            if (!commandRunning()) {
                TriggerFlow want;
                if (gamepad1.right_trigger > TRIGGER_DEADBAND) {
                    want = TriggerFlow.IN;
                } else if (gamepad1.left_trigger > TRIGGER_DEADBAND || gamepad2.aWasPressed()) {
                    want = TriggerFlow.OUT;
                } else {
                    want = TriggerFlow.OFF;
                }
                if (want != lastTriggerFlow) {
                    boolean releaseToOff = want == TriggerFlow.OFF
                            && (lastTriggerFlow == TriggerFlow.IN || lastTriggerFlow == TriggerFlow.OUT);
                    boolean applyOn = want == TriggerFlow.IN || want == TriggerFlow.OUT;
                    if (applyOn || releaseToOff) {
                        lastTriggerFlow = want;
                        Command flow;
                        switch (want) {
                            case IN:
                                flow = OpmodeCommands.turnOnIntakeAndTransfer(
                                        robot.intake, robot.transfer);
                                break;
                            case OUT:
                                flow = OpmodeCommands.extakeIntakeAndTransfer(
                                        robot.intake, robot.transfer);
                                break;
                            default:
                                flow = OpmodeCommands.turnOffIntakeAndTransfer(
                                        robot.intake, robot.transfer);
                                break;
                        }
                        Scheduler.schedule(flow);
                    } else {
                        // idle / after button cmd: remember OFF but don't command it
                        lastTriggerFlow = TriggerFlow.OFF;
                    }
                }
            } else {
                lastTriggerFlow = null;
            }

            Scheduler.execute();
            if (!commandRunning()) {
                suppressStickDrive = false;
            }

            // sticks off while smart collect owns Pedro teleop drive
            if (!suppressStickDrive) {
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
            }

            Pose field = robot.pinpoint.getPose().getAsCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
            OpmodeCommands.syncBallCount(robot.intakeGate);
            telemetry.addData("alliance", red ? "RED" : "BLUE");
            telemetry.addData("cmd", commandRunning() ? "RUNNING" : "off");
            telemetry.addData("balls in robot", "%d / %d",
                    robot.intakeGate.getBallCount(), SMART_COLLECT_TARGET);
            robot.intakeGate.addTelemetry();
            telemetry.addData("intake", robot.intake.getIntakeState());
            telemetry.addData("transfer", robot.transfer.getTransferState());
            telemetry.addData("lift", "%s pos=%d atTarget=%s",
                    robot.lift.getState(), robot.lift.getPosition(), robot.lift.atTarget());
            telemetry.addData("blocker", robot.blocker.getState());
            telemetry.addData("field", "(%.1f, %.1f, %.0f°)",
                    field.getX(), field.getY(), Math.toDegrees(field.getHeading()));
            telemetry.addData("time num", time);
            telemetry.update();
        }
        Scheduler.reset();
        cancelCommand();
        robot.drive.setMotorPowers(0, 0, 0, 0);
    }
}
