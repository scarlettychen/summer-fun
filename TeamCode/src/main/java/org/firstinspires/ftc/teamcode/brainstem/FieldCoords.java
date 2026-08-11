package org.firstinspires.ftc.teamcode.brainstem;

import com.pedropathing.geometry.Pose;

public final class FieldCoords {
    private FieldCoords() {}

    public static final double WALL = 72.0;

    public static double[] xyz(double x, double y, double headingDeg) {
        return new double[]{x, y, headingDeg};
    }

    public static double headingToward(double fromX, double fromY, double toX, double toY) {

        return Math.toDegrees(Math.atan2(toY - fromY, toX - fromX)) - 90.0;
    }

    public static double headingToward(double[] from, double[] to) {
        return headingToward(from[0], from[1], to[0], to[1]);
    }

    public static double ccwRadians(double teamHeadingRad) {
        return teamHeadingRad + Math.PI / 2;
    }

    public static String format(double[] p) {
        if (p == null || p.length < 2) {
            return "(?)";
        }
        double h = p.length >= 3 ? p[2] : 0;
        return String.format("(%.0f, %.0f, %.0f°)", p[0], p[1], h);
    }

    public static String format(Pose fieldPose) {
        return String.format("(%.1f, %.1f, %.0f°)",
                fieldPose.getX(),
                fieldPose.getY(),
                Math.toDegrees(fieldPose.getHeading()));
    }
}
