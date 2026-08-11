package org.firstinspires.ftc.teamcode.brainstem.vision;

public final class VisionPose {
    public final double xInches;
    public final double yInches;
    public final double headingDegrees;

    public static final VisionPose ZERO = new VisionPose(0, 0, 0);

    public VisionPose(double xInches, double yInches, double headingDegrees) {
        this.xInches = xInches;
        this.yInches = yInches;
        this.headingDegrees = headingDegrees;
    }

    public double[] toArray() {
        return new double[]{xInches, yInches, headingDegrees};
    }

    @Override
    public String toString() {
        return String.format("(%.1f, %.1f, %.0f°)", xInches, yInches, headingDegrees);
    }
}
