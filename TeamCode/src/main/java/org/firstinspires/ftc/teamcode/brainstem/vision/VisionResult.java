package org.firstinspires.ftc.teamcode.brainstem.vision;

import java.util.Collections;
import java.util.List;

public final class VisionResult {
    public final long timestampNanos;
    public final long frameNumber;

    public final double processingLatencySec;

    public final double inferenceTimeSec;
    public final List<Detection> detections;
    public final double bestConfidence;
    public final boolean hasTarget;

    public final VisionPose robotRelativePose;

    public final VisionPose targetRelativePose;
    public final String pipelineName;
    public final String debug;

    public static final VisionResult EMPTY = new VisionResult(
            0L, 0L, 0.0, 0.0,
            Collections.emptyList(),
            0.0, false,
            null, null,
            "none", "");

    public VisionResult(
            long timestampNanos,
            long frameNumber,
            double processingLatencySec,
            double inferenceTimeSec,
            List<Detection> detections,
            double bestConfidence,
            boolean hasTarget,
            VisionPose robotRelativePose,
            VisionPose targetRelativePose,
            String pipelineName,
            String debug) {
        this.timestampNanos = timestampNanos;
        this.frameNumber = frameNumber;
        this.processingLatencySec = processingLatencySec;
        this.inferenceTimeSec = inferenceTimeSec;
        this.detections = detections == null
                ? Collections.emptyList()
                : Collections.unmodifiableList(detections);
        this.bestConfidence = bestConfidence;
        this.hasTarget = hasTarget;
        this.robotRelativePose = robotRelativePose;
        this.targetRelativePose = targetRelativePose;
        this.pipelineName = pipelineName == null ? "" : pipelineName;
        this.debug = debug == null ? "" : debug;
    }

    public double ageSec(long nowNanos) {
        if (timestampNanos <= 0 || nowNanos < timestampNanos) {
            return Double.POSITIVE_INFINITY;
        }
        return (nowNanos - timestampNanos) * 1e-9;
    }

    public Detection bestDetection() {
        if (detections.isEmpty()) {
            return null;
        }
        Detection best = detections.get(0);
        for (int i = 1; i < detections.size(); i++) {
            Detection d = detections.get(i);
            if (d.area > best.area || (d.area == best.area && d.confidence > best.confidence)) {
                best = d;
            }
        }
        return best;
    }
}
