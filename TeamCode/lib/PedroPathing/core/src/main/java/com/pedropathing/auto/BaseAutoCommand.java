package com.pedropathing.auto;

/**
 * Default {@link AutoCommand} with empty {@link #end()}.
 */
public abstract class BaseAutoCommand implements AutoCommand {
    @Override
    public void end() {}
}
