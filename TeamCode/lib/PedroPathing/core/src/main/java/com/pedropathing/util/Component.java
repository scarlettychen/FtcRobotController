package com.pedropathing.util;

/**
 * Matches the BrainSTEM / TeamCode subsystem contract:
 * every hardware module is ticked once per robot loop via {@link #update()}.
 */
public interface Component {
    void reset();

    void update();

    String test();
}
