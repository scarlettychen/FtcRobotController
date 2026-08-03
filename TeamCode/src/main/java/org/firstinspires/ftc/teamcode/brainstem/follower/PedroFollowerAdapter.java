package org.firstinspires.ftc.teamcode.brainstem.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.brainstem.RoadRunnerCoordinates;
import org.firstinspires.ftc.teamcode.brainstem.RobotModel;
import org.firstinspires.ftc.teamcode.brainstem.utils.SafeSensor;

/**
 * Classic Pedro behind {@link PathFollower}.
 * Applies {@link VelocityConstraint} each tick via {@link RobotModel#setPathVelocityCeiling}.
 */
public final class PedroFollowerAdapter implements PathFollower {

    private final Follower pedroFollower;
    private final RobotModel robotModel;
    private final SafeSensor<Pose> safePose;
    private boolean holdEnd = false;
    private boolean manualDrive;
    private PathSpec activeSpec;

    public PedroFollowerAdapter(Follower pedroFollower, RobotModel robotModel) {
        this.pedroFollower = pedroFollower;
        this.robotModel = robotModel;
        Pose initial = pedroFollower.getPose();
        this.safePose = new SafeSensor<>(
                "pedroPose",
                pedroFollower::getPose,
                initial != null ? initial : new Pose());
    }

    public PedroFollowerAdapter holdEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    @Override
    public void startPath(PathSpec spec) {
        manualDrive = false;
        activeSpec = spec;
        PathChain chain = PathSpecConverter.toPedroPathChain(spec, pedroFollower);
        pedroFollower.followPath(chain, holdEnd);
    }

    @Override
    public FollowerOutput update(double poseX, double poseY, double poseHeadingDegrees) {
        safePose.read();

        double curvature = 0;
        Path current = pedroFollower.getCurrentPath();
        if (current != null) {
            curvature = current.getClosestPointCurvature();
        }

        double segmentCap = currentSegmentMaxVelocity();
        VelocityConstraint.Result limit = VelocityConstraint.getMaxVelocity(
                curvature, robotModel, segmentCap);

        if (robotModel != null && !manualDrive && pedroFollower.isBusy()) {
            robotModel.setPathVelocityCeiling(limit.maxVelocity);
        } else if (robotModel != null) {
            robotModel.clearPathVelocityCeiling();
        }

        pedroFollower.update();

        Vector corrective = pedroFollower.getCorrectiveVector();
        Vector heading = pedroFollower.getHeadingVector();
        Vector translational = pedroFollower.getTranslationalError();
        Vector velocity = pedroFollower.getVelocity();

        double vRef = 0;
        double aRef = 0;
        if (!manualDrive && robotModel != null && current != null && pedroFollower.isBusy()) {
            Vector tangent = pedroFollower.getClosestPointTangentVector();
            if (tangent != null && tangent.getMagnitude() > 1e-6) {
                Vector unitT = tangent.normalize();
                double remaining = Math.max(0.0, current.getDistanceRemaining());
                double vMax = Math.max(limit.maxVelocity, 1e-3);
                double aDec = Math.max(robotModel.profileMaxDeceleration(), 1e-3);
                double aAcc = Math.max(robotModel.profileMaxAcceleration(), 1e-3);
                double stopDist = (vMax * vMax) / (2.0 * aDec);
                vRef = remaining >= stopDist
                        ? vMax
                        : Math.sqrt(Math.max(0.0, 2.0 * aDec * remaining));
                double vMeas = velocity != null ? velocity.dot(unitT) : 0;
                aRef = Math.max(-aDec, Math.min(aAcc, (vRef - vMeas) * 2.0));
            }
        }

        return new FollowerOutput(
                corrective != null ? corrective.getXComponent() : 0,
                corrective != null ? corrective.getYComponent() : 0,
                heading != null
                        ? Math.copySign(heading.getMagnitude(), pedroFollower.getHeadingError())
                        : 0,
                pedroFollower.getPathCompletion(),
                translational != null ? translational.getMagnitude() : 0,
                curvature,
                vRef,
                aRef,
                limit.maxVelocity,
                limit.reason.name());
    }

    private double currentSegmentMaxVelocity() {
        if (activeSpec == null || activeSpec.segments.isEmpty()) {
            return 0;
        }
        int idx = (int) Math.round(pedroFollower.getCurrentPathNumber());
        if (idx < 0) {
            idx = 0;
        }
        if (idx >= activeSpec.segments.size()) {
            idx = activeSpec.segments.size() - 1;
        }
        return activeSpec.segments.get(idx).maxVelocity;
    }

    @Override
    public boolean isFinished() {
        if (manualDrive) {
            return false;
        }
        return !pedroFollower.isBusy();
    }

    @Override
    public boolean isBusy() {
        if (manualDrive) {
            return true;
        }
        return pedroFollower.isBusy();
    }

    @Override
    public void cancel() {
        manualDrive = false;
        activeSpec = null;
        if (robotModel != null) {
            robotModel.clearPathVelocityCeiling();
        }
        pedroFollower.breakFollowing();
    }

    @Override
    public double[] getFieldPose() {
        Pose pedro = safePose.read();
        Pose field = pedro.getAsCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
        return new double[]{
                field.getX(),
                field.getY(),
                Math.toDegrees(field.getHeading())
        };
    }

    @Override
    public void startManualDrive() {
        manualDrive = true;
        activeSpec = null;
        if (robotModel != null) {
            robotModel.clearPathVelocityCeiling();
        }
        pedroFollower.startTeleopDrive();
    }

    @Override
    public void setManualDrive(double forward, double strafe, double turn) {
        pedroFollower.setTeleOpDrive(forward, strafe, turn, true);
    }
}
