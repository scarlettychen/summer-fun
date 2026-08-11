package org.firstinspires.ftc.teamcode.brainstem.vision;

public final class Detection {
    public final String id;
    public final String className;
    public final double confidence;

    public final double txDegrees;

    public final double tyDegrees;

    public final double area;

    public final VisionPose robotRelative;

    public final VisionPose targetPose;

    public Detection(
            String id,
            String className,
            double confidence,
            double txDegrees,
            double tyDegrees,
            double area,
            VisionPose robotRelative,
            VisionPose targetPose) {
        this.id = id == null ? "" : id;
        this.className = className == null ? "" : className;
        this.confidence = confidence;
        this.txDegrees = txDegrees;
        this.tyDegrees = tyDegrees;
        this.area = area;
        this.robotRelative = robotRelative;
        this.targetPose = targetPose;
    }

    public Detection(
            String className,
            double confidence,
            double txDegrees,
            double tyDegrees,
            double area) {
        this("", className, confidence, txDegrees, tyDegrees, area, null, null);
    }
}
