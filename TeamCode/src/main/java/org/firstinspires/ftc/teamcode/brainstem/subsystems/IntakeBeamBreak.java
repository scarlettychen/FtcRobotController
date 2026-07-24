package org.firstinspires.ftc.teamcode.brainstem.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Component;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.brainstem.utils.HardwareNames;

/**
 * Two beam breaks across the intake (facing each other, perpendicular to the intake bar).
 * Counts a ball on clear→blocked edge of the gate.
 */
@Configurable
public class IntakeBeamBreak implements Component {

    /** If true, {@link DigitalChannel#getState()} false means beam broken (REV default). */
    public static boolean BROKEN_WHEN_FALSE = true;
    /** Require both sensors broken to treat gate as blocked (reduces noise). */
    public static boolean REQUIRE_BOTH = true;
    /** Min time between counted balls (ms). */
    public static double DEBOUNCE_MS = 120.0;

    private final Telemetry telemetry;
    private final DigitalChannel beamA;
    private final DigitalChannel beamB;

    private boolean prevBlocked;
    private int ballCount;
    private long lastCountMs;

    public IntakeBeamBreak(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        beamA = hwMap.get(DigitalChannel.class, HardwareNames.intakeBeamA);
        beamB = hwMap.get(DigitalChannel.class, HardwareNames.intakeBeamB);
        beamA.setMode(DigitalChannel.Mode.INPUT);
        beamB.setMode(DigitalChannel.Mode.INPUT);
        prevBlocked = isGateBlocked();
        ballCount = 0;
        lastCountMs = 0;
    }

    private boolean isBroken(DigitalChannel beam) {
        boolean state = beam.getState();
        return BROKEN_WHEN_FALSE ? !state : state;
    }

    /** True while a ball is interrupting the intake gate. */
    public boolean isGateBlocked() {
        boolean a = isBroken(beamA);
        boolean b = isBroken(beamB);
        return REQUIRE_BOTH ? (a && b) : (a || b);
    }

    public boolean isBeamABroken() {
        return isBroken(beamA);
    }

    public boolean isBeamBBroken() {
        return isBroken(beamB);
    }

    public int getBallCount() {
        return ballCount;
    }

    public void resetCount() {
        ballCount = 0;
        lastCountMs = 0;
        prevBlocked = isGateBlocked();
    }

    public void setBallCount(int count) {
        ballCount = Math.max(0, count);
    }

    @Override
    public void reset() {
        resetCount();
    }

    @Override
    public void update() {
        boolean blocked = isGateBlocked();
        long now = System.currentTimeMillis();
        if (blocked && !prevBlocked && (now - lastCountMs) >= DEBOUNCE_MS) {
            ballCount++;
            lastCountMs = now;
        }
        prevBlocked = blocked;
    }

    @Override
    public String test() {
        return "";
    }

    public void addTelemetry() {
        telemetry.addData("intake gate", "%s  a=%s b=%s",
                isGateBlocked() ? "BLOCKED" : "clear",
                isBeamABroken() ? "brk" : "ok",
                isBeamBBroken() ? "brk" : "ok");
        telemetry.addData("balls counted", ballCount);
    }
}
