package org.firstinspires.ftc.teamcode.brainstem.vision.pipelines;

import org.firstinspires.ftc.teamcode.brainstem.vision.VisionPipeline;

public final class OpenCvVisionPipeline implements VisionPipeline {

    private final String label;

    public OpenCvVisionPipeline() {
        this("opencv");
    }

    public OpenCvVisionPipeline(String label) {
        this.label = label == null ? "opencv" : label;
    }

    @Override
    public String name() {
        return label;
    }

    @Override
    public void open() {

    }

    @Override
    public void close() {

    }

    @Override
    public PipelineOutput process(long frameNumber) {

        return PipelineOutput.EMPTY;
    }
}
