package com.pedropathing.auto;

import java.util.function.BooleanSupplier;

/**
 * Runs exactly one of two {@link AutoCommand}s based on a condition evaluated once at
 * {@link #initialize()} (WPILib-style {@code ConditionalCommand}).
 *
 * <p>The unselected branch is never initialized.
 *
 * <pre>{@code
 * conditional(
 *     robot::hasGamePiece,
 *     scoreGamePiece(),
 *     retryIntake()
 * );
 * }</pre>
 */
public class ConditionalCommand extends BaseAutoCommand {
    private final BooleanSupplier condition;
    private final AutoCommand trueCommand;
    private final AutoCommand falseCommand;

    /** Selected branch after {@link #initialize()}; null until then. */
    private AutoCommand selected;

    /**
     * @param condition   evaluated once in {@link #initialize()}; {@code null} → false
     * @param trueCommand run when condition is true; {@code null} → instant no-op
     * @param falseCommand run when condition is false; {@code null} → instant no-op
     */
    public ConditionalCommand(
            BooleanSupplier condition,
            AutoCommand trueCommand,
            AutoCommand falseCommand
    ) {
        this.condition = condition != null ? condition : () -> false;
        this.trueCommand = trueCommand != null ? trueCommand : FunctionalCommand.instant(() -> {});
        this.falseCommand = falseCommand != null ? falseCommand : FunctionalCommand.instant(() -> {});
    }

    /** Factory matching {@link ActionLibrary#conditional}. */
    public static AutoCommand conditional(
            BooleanSupplier condition,
            AutoCommand onTrue,
            AutoCommand onFalse
    ) {
        return new ConditionalCommand(condition, onTrue, onFalse);
    }

    @Override
    public void initialize() {
        selected = condition.getAsBoolean() ? trueCommand : falseCommand;
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
}
