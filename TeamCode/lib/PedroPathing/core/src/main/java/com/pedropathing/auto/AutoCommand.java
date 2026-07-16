package com.pedropathing.auto;

/**
 * Beginner-facing autonomous action unit.
 * Schedulers call: initialize → execute (loop) → isFinished? → end.
 */
public interface AutoCommand {
    void initialize();

    void execute();

    boolean isFinished();

    /** Called when the command completes or is cancelled. */
    void end();
}
