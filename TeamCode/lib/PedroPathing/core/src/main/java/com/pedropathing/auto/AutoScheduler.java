package com.pedropathing.auto;

/**
 * Tiny cooperative scheduler for {@link AutoCommand}s.
 * Call {@link #run()} once per OpMode loop.
 */
public class AutoScheduler {
    private AutoCommand current;
    private boolean initialized;

    public void schedule(AutoCommand command) {
        if (current != null) {
            current.end();
        }
        current = command;
        initialized = false;
    }

    public void cancel() {
        if (current != null) {
            current.end();
            current = null;
            initialized = false;
        }
    }

    public boolean isFinished() {
        return current == null;
    }

    public boolean isRunning() {
        return current != null;
    }

    public void run() {
        if (current == null) return;
        if (!initialized) {
            current.initialize();
            initialized = true;
        }
        current.execute();
        if (current.isFinished()) {
            current.end();
            current = null;
            initialized = false;
        }
    }
}
