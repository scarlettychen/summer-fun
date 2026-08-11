package org.firstinspires.ftc.teamcode.brainstem.vision.pipelines;

import org.firstinspires.ftc.teamcode.brainstem.vision.VisionPipeline;

public final class TensorFlowVisionPipeline implements VisionPipeline {

    private final String modelName;

    public TensorFlowVisionPipeline() {
        this("tflite");
    }

    public TensorFlowVisionPipeline(String modelName) {
        this.modelName = modelName == null ? "tflite" : modelName;
    }

    @Override
    public String name() {
        return modelName;
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
