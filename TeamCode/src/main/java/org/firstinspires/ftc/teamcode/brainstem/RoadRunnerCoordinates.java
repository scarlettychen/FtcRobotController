package org.firstinspires.ftc.teamcode.brainstem;

import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.FTCCoordinates;

/**
 * Team field grid for pose arrays {@code {x, y, headingDeg}}.
 *
 * <p>Center origin, walls at ±72. <b>0° = into the field from the −Y wall (+Y)</b>.
 * Degrees increase counter-clockwise: 90° = −X, 180° = −Y, −90° = +X.
 *
 * <p>Wraps Pedro {@link FTCCoordinates}: team heading = FTC heading − 90°.
 *
 * @see FieldCoords
 */
public enum RoadRunnerCoordinates implements CoordinateSystem {
    INSTANCE;

    private static final double TEAM_ZERO_OFFSET = Math.PI / 2; // team 0 = FTC +Y

    @Override
    public Pose convertToPedro(Pose pose) {
        // team (0=+Y CCW) → FTC (0=+X CCW)
        Pose ftc = new Pose(pose.getX(), pose.getY(), pose.getHeading() + TEAM_ZERO_OFFSET);
        return FTCCoordinates.INSTANCE.convertToPedro(ftc);
    }

    @Override
    public Pose convertFromPedro(Pose pose) {
        Pose ftc = FTCCoordinates.INSTANCE.convertFromPedro(pose);
        return new Pose(ftc.getX(), ftc.getY(), ftc.getHeading() - TEAM_ZERO_OFFSET, INSTANCE);
    }
}
