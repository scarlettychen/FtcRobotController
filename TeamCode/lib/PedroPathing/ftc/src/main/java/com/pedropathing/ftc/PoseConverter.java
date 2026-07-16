package com.pedropathing.ftc;

import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Single gateway for field poses ↔ Pedro / SDK {@link Pose2D}.
 *
 * <p>Call {@link #useFTCCoordinates()} once at robot init (also done by {@link FollowerBuilder#build()})
 * so TeamCode arrays, Limelight botpose, and BrainSTEM pushes are interpreted as FTC field coords
 * (corner origin) and converted to Pedro before paths / following.
 */
public final class PoseConverter {
    private PoseConverter() {}

    /** Interpret all field numbers as FTC standard (corner origin). */
    public static void useFTCCoordinates() {
        Pose.setFieldCoordinateSystem(FTCCoordinates.INSTANCE);
    }

    /** DECODE / inverted FTC field. */
    public static void useInvertedFTCCoordinates() {
        Pose.setFieldCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
    }

    /** No conversion — field numbers are already Pedro. */
    public static void usePedroCoordinates() {
        Pose.setFieldCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    public static void setFieldCoordinateSystem(CoordinateSystem coordinateSystem) {
        Pose.setFieldCoordinateSystem(coordinateSystem);
    }

    /** Field inches + radians → Pedro {@link Pose}. */
    public static Pose fromField(double x, double y, double headingRadians) {
        return Pose.fromField(x, y, headingRadians);
    }

    /** Field inches + degrees → Pedro {@link Pose}. */
    public static Pose fromFieldDegrees(double x, double y, double headingDegrees) {
        return Pose.fromFieldDegrees(x, y, headingDegrees);
    }

    /**
     * TeamCode-style {@code {x, y, headingDegrees}} → Pedro pose.
     */
    public static Pose fromArray(double[] pose) {
        if (pose == null || pose.length < 2) {
            throw new IllegalArgumentException("Pose array needs at least x,y");
        }
        double headingDeg = pose.length >= 3 ? pose[2] : 0;
        return fromFieldDegrees(pose[0], pose[1], headingDeg);
    }

    /**
     * Converts a Pose to a Pose2D in the desired coordinate system.
     */
    public static Pose2D poseToPose2D(Pose pose, CoordinateSystem desiredCoordinateSystem) {
        Pose converted = pose.getAsCoordinateSystem(desiredCoordinateSystem);
        return new Pose2D(
                DistanceUnit.INCH,
                converted.getX(),
                converted.getY(),
                AngleUnit.RADIANS,
                converted.getHeading()
        );
    }

    /**
     * SDK Pose2D tagged with {@code coordinateSystem}, returned as Pedro for Follower math.
     */
    public static Pose pose2DToPose(Pose2D pose2d, CoordinateSystem coordinateSystem) {
        Pose tagged = new Pose(
                pose2d.getX(DistanceUnit.INCH),
                pose2d.getY(DistanceUnit.INCH),
                pose2d.getHeading(AngleUnit.RADIANS),
                coordinateSystem
        );
        return tagged.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}
