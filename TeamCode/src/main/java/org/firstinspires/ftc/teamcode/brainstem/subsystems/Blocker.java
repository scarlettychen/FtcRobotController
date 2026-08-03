package org.firstinspires.ftc.teamcode.brainstem.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Component;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.brainstem.FieldCoords;
import org.firstinspires.ftc.teamcode.brainstem.RoadRunnerCoordinates;
import org.firstinspires.ftc.teamcode.brainstem.utils.HardwareNames;

@Configurable
public class Blocker implements Component {

    public static int PWM_LOWER = 1300;
    public static int PWM_UPPER = 1500;

    public static double UP_POS = 1.0;
    public static double DOWN_POS = 0.0;

    // where the blocker sits on the bot: +fwd / +left from center (inches)
    public static double OFFSET_FORWARD_IN = 0.0;
    public static double OFFSET_LEFT_IN = 0.0;

    private final ServoImplEx servo;
    private final Telemetry telemetry;

    public enum ServoState {
        UP,
        DOWN
    }

    private ServoState state;

    public Blocker(HardwareMap map, Telemetry telemetry) {
        servo = map.get(ServoImplEx.class, HardwareNames.blockerServo);
        servo.setPwmRange(new PwmControl.PwmRange(PWM_LOWER, PWM_UPPER));
        state = ServoState.DOWN;
        this.telemetry = telemetry;
    }

    public void setOpen() {
        state = ServoState.UP;
    }

    public void setDown() {
        state = ServoState.DOWN;
    }

    public void setState(ServoState state) {
        this.state = state;
    }

    public ServoState getState() {
        return state;
    }

    // blocker field pose in rr inches
    public Pose blockerFieldPose(Pose robotPedroPose) {
        Pose field = robotPedroPose.getAsCoordinateSystem(RoadRunnerCoordinates.INSTANCE);
        // team 0° = +Y CCW; body→field uses standard 0=+X frame
        double hCcw = FieldCoords.ccwRadians(field.getHeading());
        double cos = Math.cos(hCcw);
        double sin = Math.sin(hCcw);
        // robot +fwd/+left → rr field (0° = +x)
        double x = field.getX() + OFFSET_FORWARD_IN * cos - OFFSET_LEFT_IN * sin;
        double y = field.getY() + OFFSET_FORWARD_IN * sin + OFFSET_LEFT_IN * cos;
        return new Pose(x, y, field.getHeading(), RoadRunnerCoordinates.INSTANCE);
    }

    // true if the blocker (not robot center) is inside the goal box
    public boolean isOverRegion(
            Pose robotPedroPose,
            double regionX,
            double regionY,
            double halfWidth,
            double halfHeight
    ) {
        Pose blocker = blockerFieldPose(robotPedroPose);
        return Math.abs(blocker.getX() - regionX) <= halfWidth
                && Math.abs(blocker.getY() - regionY) <= halfHeight;
    }

    /** FieldCoords pose {@code {x,y,headingDeg}} — no Pedro types at the call site. */
    public boolean isOverRegionField(
            double[] fieldPose,
            double regionX,
            double regionY,
            double halfWidth,
            double halfHeight
    ) {
        double hRad = Math.toRadians(fieldPose[2]);
        double hCcw = FieldCoords.ccwRadians(hRad);
        double cos = Math.cos(hCcw);
        double sin = Math.sin(hCcw);
        double bx = fieldPose[0] + OFFSET_FORWARD_IN * cos - OFFSET_LEFT_IN * sin;
        double by = fieldPose[1] + OFFSET_FORWARD_IN * sin + OFFSET_LEFT_IN * cos;
        return Math.abs(bx - regionX) <= halfWidth && Math.abs(by - regionY) <= halfHeight;
    }

    @Override
    public void reset() {}

    @Override
    public void update() {
        switch (state) {
            case UP:
                servo.setPosition(UP_POS);
                break;
            case DOWN:
                servo.setPosition(DOWN_POS);
                break;
        }
    }

    @Override
    public String test() {
        return "";
    }
}
