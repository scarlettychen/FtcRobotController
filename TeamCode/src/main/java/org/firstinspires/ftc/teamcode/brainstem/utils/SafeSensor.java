package org.firstinspires.ftc.teamcode.brainstem.utils;

import java.util.function.Supplier;

/**
 * Wraps a single sensor read so a null return, thrown exception, or garbage I2C
 * frame never crashes the OpMode loop. On failure it hands back the last known-good
 * value (or your chosen fallback if there isn't one yet) and flags itself unhealthy
 * so you can surface it on telemetry or the Death Logger.
 *
 * One instance per sensor. Generic over T so it works for distance (Double),
 * color (NormalizedRGBA), or anything else you poll every loop.
 */
public class SafeSensor<T> {

    private final String name;
    private final Supplier<T> reader;

    private T lastGoodValue;
    private boolean healthy = true;
    private int consecutiveFailures = 0;
    private long lastFailureTimestamp = 0;

    public SafeSensor(String name, Supplier<T> reader, T fallback) {
        this.name = name;
        this.reader = reader;
        this.lastGoodValue = fallback;
    }

    /** Call this instead of reading the sensor directly. This method never throws. */
    public T read() {
        try {
            T value = reader.get();
            if (value == null) {
                throw new NullPointerException(name + " returned null");
            }
            lastGoodValue = value;
            healthy = true;
            consecutiveFailures = 0;
        } catch (Exception e) {
            consecutiveFailures++;
            healthy = false;
            lastFailureTimestamp = System.currentTimeMillis();
            // fall through -- lastGoodValue is returned below, control stays with the driver
        }
        return lastGoodValue;
    }

    public boolean isHealthy() {
        return healthy;
    }

    public int getConsecutiveFailures() {
        return consecutiveFailures;
    }

    public String getName() {
        return name;
    }

    public long getLastFailureTimestamp() {
        return lastFailureTimestamp;
    }
}

/*
Usage in your OpMode:

SafeSensor<Double> frontDistance = new SafeSensor<>(
    "frontDistance",
    () -> distanceSensor.getDistance(DistanceUnit.CM),
    999.0 // fallback: treat a dead sensor as "nothing detected" rather than 0 (which could
          // look like a real obstacle right in front of the robot and freeze auto-behavior)
);

// every loop:
double d = frontDistance.read();
telemetry.addData(frontDistance.getName(), frontDistance.isHealthy() ? d : "DOWN (" + frontDistance.getConsecutiveFailures() + " fails)");

Wrap every sensor you poll this way -- color sensors, the IMU read itself, encoders on
non-motor ports, whatever your team touches every loop. The pattern is the same: never
call the raw sensor method directly from your OpMode, always go through a SafeSensor.
*/