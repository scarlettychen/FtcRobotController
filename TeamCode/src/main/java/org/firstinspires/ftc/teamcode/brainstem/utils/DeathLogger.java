package org.firstinspires.ftc.teamcode.brainstem.utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * Background "black box" logger for post-DC forensics.
 *
 * Design: log() is cheap and non-blocking -- it just formats a string and drops it on
 * a queue. A dedicated daemon thread pulls off the queue and does the actual (slow)
 * file write. This matters because writing to flash on every single loop iteration
 * from the main thread is exactly the kind of thing that can itself cause the loop
 * time spikes you're trying to catch -- the logger must not become the disease it's
 * meant to diagnose.
 */
public class DeathLogger {

    private static final String LOG_DIR = "/sdcard/FIRST/logs/";

    private final BlockingQueue<String> queue = new LinkedBlockingQueue<>();
    private volatile boolean running = false;
    private Thread writerThread;
    private File logFile;

    /** Call once from init(). Creates a new timestamped log file for this run. */
    public void start(String opModeName) {
        new File(LOG_DIR).mkdirs();
        logFile = new File(LOG_DIR + opModeName + "_" + System.currentTimeMillis() + ".txt");
        running = true;

        writerThread = new Thread(() -> {
            try (BufferedWriter writer = new BufferedWriter(new FileWriter(logFile, true))) {
                while (running || !queue.isEmpty()) {
                    String line = queue.poll(200, TimeUnit.MILLISECONDS);
                    if (line != null) {
                        writer.write(line);
                        writer.newLine();
                        writer.flush(); // flush per line: if the hub browns out mid-match,
                        // you keep everything written up to that point
                    }
                }
            } catch (IOException | InterruptedException e) {
                // the logger itself must never crash the OpMode -- swallow and stop quietly
            }
        }, "DeathLogger-Writer");
        writerThread.setDaemon(true);
        writerThread.start();
    }

    /** Cheap -- call every loop iteration. Never blocks, never throws. */
    public void log(double loopTimeMs, double batteryVoltage, String activeMechanisms) {
        if (!running) return;
        String line = String.format("%d,loopMs=%.1f,battery=%.2f,mechanisms=%s",
                System.currentTimeMillis(), loopTimeMs, batteryVoltage, activeMechanisms);
        queue.offer(line);
    }

    /** Call from stop(). Drains the remaining queue before shutting the thread down. */
    public void stop() {
        running = false;
        if (writerThread != null) {
            try {
                writerThread.join(500);
            } catch (InterruptedException ignored) {
            }
        }
    }

    public File getLogFile() {
        return logFile;
    }
}

/*
Usage in your OpMode:

private final DeathLogger deathLogger = new DeathLogger();

@Override
public void init() {
    deathLogger.start("TeleOp_BIOBUZZ");
}

@Override
public void loop() {
    long loopStart = System.currentTimeMillis();

    // ... your existing loop logic (drive, vision, mechanisms) ...

    double loopTimeMs = System.currentTimeMillis() - loopStart;
    double battery = hardwareMap.voltageSensor.iterator().next().getVoltage();
    deathLogger.log(loopTimeMs, battery, "intake=" + intakeState + ",lift=" + liftState);
}

@Override
public void stop() {
    deathLogger.stop();
}

Pull the file off the Control Hub afterward via the Manage webpage's file browser, or
adb pull /sdcard/FIRST/logs/. Loop-time spikes and voltage sags right before a DC
timestamp are your two main things to look for.
*/
