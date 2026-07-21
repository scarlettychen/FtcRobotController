package com.pedropathing.auto;

import java.util.function.BooleanSupplier;

/**
 * Lightweight validation for auton actions (intake present, shooter at speed, etc.).
 *
 * <pre>{@code
 * validate(robot::hasGamePiece, continueScoring(), retryIntake());
 *
 * waitUntilValidated(shooter::isAtVelocity, 1.5);
 * }</pre>
 */
public class ValidationCommand extends BaseAutoCommand {
    private final BooleanSupplier condition;
    private final AutoCommand onSuccess;
    private final AutoCommand onFailure;

    private AutoCommand selected;

    /**
     * @param condition  evaluated once at {@link #initialize()}; {@code null} → false
     * @param onSuccess  run when true; {@code null} → instant no-op
     * @param onFailure  run when false; {@code null} → instant no-op
     */
    public ValidationCommand(
            BooleanSupplier condition,
            AutoCommand onSuccess,
            AutoCommand onFailure
    ) {
        this.condition = condition != null ? condition : () -> false;
        this.onSuccess = onSuccess != null ? onSuccess : FunctionalCommand.instant(() -> {});
        this.onFailure = onFailure != null ? onFailure : FunctionalCommand.instant(() -> {});
    }

    /**
     * Evaluate {@code condition} once after start; run {@code onSuccess} or {@code onFailure}.
     * Only the selected branch is initialized.
     */
    public static AutoCommand validate(
            BooleanSupplier condition,
            AutoCommand onSuccess,
            AutoCommand onFailure
    ) {
        return new ValidationCommand(condition, onSuccess, onFailure);
    }

    /**
     * Wait until {@code condition} is true, or finish on timeout (auton continues).
     * Timing style matches {@link FunctionalCommand#waitSeconds(double)}.
     */
    public static AutoCommand waitUntilValidated(BooleanSupplier condition, double timeoutSeconds) {
        return new WaitUntilValidated(condition, timeoutSeconds);
    }

    @Override
    public void initialize() {
        boolean ok = condition.getAsBoolean();
        if (ok) {
            logResult("PASS");
            selected = onSuccess;
        } else {
            logResult("FAILED");
            selected = onFailure;
        }
        selected.initialize();
    }

    @Override
    public void execute() {
        if (selected != null) {
            selected.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return selected != null && selected.isFinished();
    }

    @Override
    public void end() {
        if (selected != null) {
            selected.end();
        }
    }

    static void logResult(String result) {
        System.out.println("Validation:");
        System.out.println("    " + result);
    }

    /**
     * Polls {@code condition} each loop until true or {@code timeoutSeconds} elapses.
     * Uses the same nanoTime deadline pattern as {@link FunctionalCommand#waitSeconds}.
     */
    private static final class WaitUntilValidated extends BaseAutoCommand {
        private final BooleanSupplier condition;
        private final long timeoutNanos;
        private long deadlineNanos;
        private boolean done;

        WaitUntilValidated(BooleanSupplier condition, double timeoutSeconds) {
            this.condition = condition != null ? condition : () -> false;
            this.timeoutNanos = (long) (Math.max(0, timeoutSeconds) * 1_000_000_000L);
        }

        @Override
        public void initialize() {
            done = false;
            deadlineNanos = System.nanoTime() + timeoutNanos;
        }

        @Override
        public void execute() {
            if (done) {
                return;
            }
            if (condition.getAsBoolean()) {
                logResult("PASS");
                done = true;
            } else if (System.nanoTime() >= deadlineNanos) {
                logResult("TIMEOUT");
                done = true;
            }
        }

        @Override
        public boolean isFinished() {
            return done;
        }
    }
}
