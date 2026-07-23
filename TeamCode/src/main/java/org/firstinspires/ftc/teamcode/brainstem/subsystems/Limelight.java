package org.firstinspires.ftc.teamcode.brainstem.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Component;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;


@Configurable
public class Limelight implements Component {

    // cam height off the floor (inches)
    public static double CAMERA_HEIGHT_IN = 12.0;
    // pitch: neg = pointed at the floor
    public static double CAMERA_PITCH_DEG = -25.0;
    // where the cam sits on the bot: +fwd / +left from center
    public static double CAMERA_FORWARD_IN = 0.0;
    public static double CAMERA_LEFT_IN = 0.0;
    // skip tiny / far junk
    public static double MIN_AREA = 0.001;
    public static double MIN_CONFIDENCE = 0.3;
    public static double MAX_RANGE_IN = 72.0;
    public static double MIN_RANGE_IN = 6.0;
    // blank = any class (detector only does balls anyway)
    public static String CLASS_FILTER = "";

    // ball chase pid (teleop powers)
    public static double CHASE_KP_TX = 0.025;
    public static double CHASE_KI_TX = 0.0;
    public static double CHASE_KD_TX = 0.001;
    public static double CHASE_KP_RANGE = 0.04;
    public static double CHASE_KI_RANGE = 0.0;
    public static double CHASE_KD_RANGE = 0.0;
    public static double CHASE_MAX_FORWARD = 0.90;
    public static double CHASE_MAX_TURN = 0.7;
    // done when |tx| small + range under stop
    public static double CHASE_TX_TOL_DEG = 3.0;
    public static double CHASE_STOP_RANGE_IN = 10.0;
    public static int CHASE_LOST_FRAMES = 15;

    private final Telemetry telemetry;
    private final Limelight3A lime;

    private BallDetection closest;

    public static final class BallDetection {
        public final String className;
        public final double confidence;
        public final double txDeg;
        public final double tyDeg;
        public final double area;

        public BallDetection(
                String className, double confidence, double txDeg, double tyDeg, double area) {
            this.className = className;
            this.confidence = confidence;
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.area = area;
        }
    }

    public Limelight(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        lime = map.get(Limelight3A.class, "limelight");
        lime.setPollRateHz(100);
        lime.pipelineSwitch(4);
        lime.start();
    }

    // biggest blob from last frame, or null if nothing
    public BallDetection getClosestBall() {
        return closest;
    }

    public boolean hasBall() {
        return closest != null;
    }

    // guess rr field pose of closest ball {x,y,headingDeg}, or null
    public double[] estimateClosestBallFieldPose(Pose robotPedroPose) {
        if (closest == null || robotPedroPose == null) {
            return null;
        }
        return estimateBallFieldPose(robotPedroPose, closest);
    }

    // floor ball range from pitch+ty, then robot→rr with the x/y flip
    public double[] estimateBallFieldPose(Pose robotPedroPose, BallDetection ball) {
        double range = estimateRangeInches(ball.tyDeg);
        if (Double.isNaN(range)) {
            return null;
        }
        range = Math.max(MIN_RANGE_IN, Math.min(MAX_RANGE_IN, range));

        // limelight +tx = target right of crosshair
        double txRad = Math.toRadians(ball.txDeg);
        double forwardFromCamera = range * Math.cos(txRad);
        double rightFromCamera = range * Math.sin(txRad);
        double forward = CAMERA_FORWARD_IN + forwardFromCamera;
        double left = CAMERA_LEFT_IN - rightFromCamera;

        Pose field = robotPedroPose.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        double h = field.getHeading();
        double cos = Math.cos(h);
        double sin = Math.sin(h);
        // same rr x/y flip as blocker offsets
        double alongHeading = forward * cos - left * sin;
        double acrossHeading = forward * sin + left * cos;
        double x = field.getX() + acrossHeading;
        double y = field.getY() + alongHeading;
        return new double[]{x, y, Math.toDegrees(h)};
    }

    public double estimateRangeInches(double tyDeg) {
        double angleDownRad = Math.toRadians(-(CAMERA_PITCH_DEG + tyDeg));
        if (angleDownRad <= 1e-3) {
            return Double.NaN;
        }
        return CAMERA_HEIGHT_IN / Math.tan(angleDownRad);
    }

    public double estimateClosestRangeInches() {
        if (closest == null) {
            return Double.NaN;
        }
        return estimateRangeInches(closest.tyDeg);
    }

    @Override
    public void reset() {
        closest = null;
    }

    @Override
    public void update() {
        closest = null;
        LLResult result = lime.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("balls", 0);
            return;
        }

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        telemetry.addData("balls", detections == null ? 0 : detections.size());
        if (detections == null || detections.isEmpty()) {
            return;
        }

        BallDetection best = null;
        for (LLResultTypes.DetectorResult d : detections) {
            if (d.getTargetArea() < MIN_AREA) {
                continue;
            }
            if (d.getConfidence() < MIN_CONFIDENCE) {
                continue;
            }
            String name = d.getClassName() == null ? "" : d.getClassName();
            if (CLASS_FILTER != null
                    && !CLASS_FILTER.isEmpty()
                    && !name.toLowerCase().contains(CLASS_FILTER.toLowerCase())) {
                continue;
            }
            if (best == null || d.getTargetArea() > best.area) {
                best = new BallDetection(
                        name,
                        d.getConfidence(),
                        d.getTargetXDegrees(),
                        d.getTargetYDegrees(),
                        d.getTargetArea());
            }
        }
        closest = best;
        if (closest != null) {
            telemetry.addData("closest", "%s area=%.3f tx=%.1f",
                    closest.className, closest.area, closest.txDeg);
        }
    }

    @Override
    public String test() {
        return "Limelight";
    }
}
