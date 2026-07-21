package com.pedropathing.auto;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Re-runs a fresh {@link AutoCommand} from a {@link Supplier} until a success condition
 * passes or {@code maxAttempts} is exhausted. Compatible with {@code sequence}/{@code parallel}
 * — no sleeps, no nested scheduler.
 *
 * <pre>{@code
 * retry(
 *     () -> intakePixel(),
 *     robot::hasPixel,
 *     2
 * );
 * }</pre>
 *
 * <p>Each attempt gets a <em>new</em> command instance from the supplier. After an attempt's
 * {@link AutoCommand#isFinished()} / {@link AutoCommand#end()}, {@code successCondition} is
 * checked once. On failure with attempts left, the next instance is started immediately.
 * If all attempts fail, this command still finishes so the auton can continue.
 */
public class RetryCommand extends BaseAutoCommand {
    private final Supplier<AutoCommand> commandSupplier;
    private final BooleanSupplier successCondition;
    private final int maxAttempts;

    private AutoCommand current;
    private int attempt;
    private boolean done;

    /**
     * @param commandSupplier  produces a fresh command per attempt; {@code null} → no-op attempts
     * @param successCondition checked after each attempt ends; {@code null} → always false
     * @param maxAttempts      minimum 1
     */
    public RetryCommand(
            Supplier<AutoCommand> commandSupplier,
            BooleanSupplier successCondition,
            int maxAttempts
    ) {
        this.commandSupplier = commandSupplier != null
                ? commandSupplier
                : () -> FunctionalCommand.instant(() -> {});
        this.successCondition = successCondition != null ? successCondition : () -> false;
        this.maxAttempts = Math.max(1, maxAttempts);
    }

    /** Factory matching {@link ActionLibrary#retry}. */
    public static AutoCommand retry(
            Supplier<AutoCommand> command,
            BooleanSupplier successCondition,
            int maxAttempts
    ) {
        return new RetryCommand(command, successCondition, maxAttempts);
    }

    @Override
    public void initialize() {
        done = false;
        attempt = 0;
        current = null;
        startNextAttempt();
    }

    @Override
    public void execute() {
        if (done || current == null) {
            return;
        }

        current.execute();
        if (!current.isFinished()) {
            return;
        }

        current.end();
        current = null;

        if (successCondition.getAsBoolean()) {
            log("Retry succeeded");
            done = true;
            return;
        }

        if (attempt < maxAttempts) {
            startNextAttempt();
        } else {
            log("Retry failed");
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end() {
        if (current != null) {
            current.end();
            current = null;
        }
    }

    private void startNextAttempt() {
        attempt++;
        log("RetryCommand attempt " + attempt + "/" + maxAttempts);
        AutoCommand next = commandSupplier.get();
        current = next != null ? next : FunctionalCommand.instant(() -> {});
        current.initialize();
    }

    private static void log(String message) {
        System.out.println(message);
    }
}
