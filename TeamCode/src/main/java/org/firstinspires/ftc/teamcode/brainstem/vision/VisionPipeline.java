package org.firstinspires.ftc.teamcode.brainstem.vision;

import java.util.Collections;
import java.util.List;

public interface VisionPipeline {

    String name();

    void open();

    void close();

    PipelineOutput process(long frameNumber);

    default double cameraFps() {
        return 0;
    }

    final class PipelineOutput {
        public final List<Detection> detections;
        public final double inferenceTimeSec;
        public final VisionPose robotRelativePose;
        public final VisionPose targetRelativePose;
        public final String debug;

        public final int droppedWhileProcessing;

        public static final PipelineOutput EMPTY = new PipelineOutput(
                Collections.emptyList(), 0, null, null, "", 0);

        public PipelineOutput(
                List<Detection> detections,
                double inferenceTimeSec,
                VisionPose robotRelativePose,
                VisionPose targetRelativePose,
                String debug,
                int droppedWhileProcessing) {
            this.detections = detections == null
                    ? Collections.emptyList()
                    : Collections.unmodifiableList(detections);
            this.inferenceTimeSec = inferenceTimeSec;
            this.robotRelativePose = robotRelativePose;
            this.targetRelativePose = targetRelativePose;
            this.debug = debug == null ? "" : debug;
            this.droppedWhileProcessing = Math.max(0, droppedWhileProcessing);
        }
    }
}
