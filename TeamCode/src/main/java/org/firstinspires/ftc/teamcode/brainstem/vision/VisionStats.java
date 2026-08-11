package org.firstinspires.ftc.teamcode.brainstem.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class VisionStats {
    public final double visionFps;
    public final double avgLatencySec;
    public final double avgInferenceSec;
    public final double cameraFps;
    public final long droppedFrames;
    public final long processedFrames;
    public final long lastTimestampNanos;
    public final boolean threadAlive;
    public final boolean hasTarget;
    public final double bestConfidence;
    public final String pipelineName;

    public VisionStats(
            double visionFps,
            double avgLatencySec,
            double avgInferenceSec,
            double cameraFps,
            long droppedFrames,
            long processedFrames,
            long lastTimestampNanos,
            boolean threadAlive,
            boolean hasTarget,
            double bestConfidence,
            String pipelineName) {
        this.visionFps = visionFps;
        this.avgLatencySec = avgLatencySec;
        this.avgInferenceSec = avgInferenceSec;
        this.cameraFps = cameraFps;
        this.droppedFrames = droppedFrames;
        this.processedFrames = processedFrames;
        this.lastTimestampNanos = lastTimestampNanos;
        this.threadAlive = threadAlive;
        this.hasTarget = hasTarget;
        this.bestConfidence = bestConfidence;
        this.pipelineName = pipelineName;
    }

    public void addTelemetry(Telemetry telemetry, String prefix) {
        String p = prefix == null || prefix.isEmpty() ? "vision" : prefix;
        telemetry.addData(p + " fps", "%.1f", visionFps);
        telemetry.addData(p + " latency ms", "%.1f", avgLatencySec * 1000.0);
        telemetry.addData(p + " infer ms", "%.1f", avgInferenceSec * 1000.0);
        telemetry.addData(p + " cam fps", "%.1f", cameraFps);
        telemetry.addData(p + " dropped", droppedFrames);
        telemetry.addData(p + " frames", processedFrames);
        telemetry.addData(p + " hasTarget", hasTarget);
        telemetry.addData(p + " conf", "%.2f", bestConfidence);
        telemetry.addData(p + " thread", threadAlive ? "ALIVE" : "dead");
        telemetry.addData(p + " pipe", pipelineName);
        if (lastTimestampNanos > 0) {
            telemetry.addData(p + " age ms", "%.0f",
                    (System.nanoTime() - lastTimestampNanos) * 1e-6);
        }
    }
}
