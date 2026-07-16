package com.pedropathing.auto;

import java.util.function.BooleanSupplier;

/**
 * Lambda / delay / one-shot {@link AutoCommand}. Prefer factories over subclasses.
 */
public class FunctionalCommand extends BaseAutoCommand {
    private final Runnable onInit;
    private final Runnable onExecute;
    private final BooleanSupplier finished;
    private final Runnable onEnd;

    public FunctionalCommand(Runnable onInit, Runnable onExecute, BooleanSupplier finished, Runnable onEnd) {
        this.onInit = onInit == null ? () -> {} : onInit;
        this.onExecute = onExecute == null ? () -> {} : onExecute;
        this.finished = finished == null ? () -> true : finished;
        this.onEnd = onEnd == null ? () -> {} : onEnd;
    }

    public static FunctionalCommand runUntil(Runnable execute, BooleanSupplier finished) {
        return new FunctionalCommand(null, execute, finished, null);
    }

    /** One-shot action that finishes after one execute. */
    public static AutoCommand instant(Runnable action) {
        return new Instant(action);
    }

    /** Wall-clock delay in seconds. */
    public static AutoCommand waitSeconds(double seconds) {
        return new Wait(seconds);
    }

    @Override
    public void initialize() {
        onInit.run();
    }

    @Override
    public void execute() {
        onExecute.run();
    }

    @Override
    public boolean isFinished() {
        return finished.getAsBoolean();
    }

    @Override
    public void end() {
        onEnd.run();
    }

    private static final class Instant extends BaseAutoCommand {
        private final Runnable action;
        private boolean done;

        Instant(Runnable action) {
            this.action = action == null ? () -> {} : action;
        }

        @Override
        public void initialize() {
            done = false;
        }

        @Override
        public void execute() {
            if (!done) {
                action.run();
                done = true;
            }
        }

        @Override
        public boolean isFinished() {
            return done;
        }
    }

    private static final class Wait extends BaseAutoCommand {
        private final long durationNanos;
        private long endNanos;

        Wait(double seconds) {
            this.durationNanos = (long) (Math.max(0, seconds) * 1_000_000_000L);
        }

        @Override
        public void initialize() {
            endNanos = System.nanoTime() + durationNanos;
        }

        @Override
        public void execute() {}

        @Override
        public boolean isFinished() {
            return System.nanoTime() >= endNanos;
        }
    }
}
