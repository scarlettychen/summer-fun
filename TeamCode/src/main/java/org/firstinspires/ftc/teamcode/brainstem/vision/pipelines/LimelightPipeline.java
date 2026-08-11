package org.firstinspires.ftc.teamcode.brainstem.vision.pipelines;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.brainstem.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.brainstem.vision.Detection;
import org.firstinspires.ftc.teamcode.brainstem.vision.VisionPipeline;
import org.firstinspires.ftc.teamcode.brainstem.vision.VisionPose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Configurable
public final class LimelightPipeline implements VisionPipeline {

    public static String HARDWARE_NAME = "limelight";
    public static int PIPELINE_INDEX = 4;
    public static int POLL_RATE_HZ = 50;

    private final HardwareMap hardwareMap;
    private Limelight3A lime;
    private long lastCaptureNanos;
    private double lastCameraFps;

    private final ArrayList<Detection> scratch = new ArrayList<>(8);

    public LimelightPipeline(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public String name() {
        return "limelight";
    }

    @Override
    public void open() {
        lime = hardwareMap.get(Limelight3A.class, HARDWARE_NAME);
        lime.setPollRateHz(POLL_RATE_HZ);
        lime.pipelineSwitch(PIPELINE_INDEX);
        lime.start();
        lastCaptureNanos = 0;
        lastCameraFps = POLL_RATE_HZ;
    }

    @Override
    public void close() {
        if (lime != null) {
            try {
                lime.stop();
            } catch (RuntimeException ignored) {
            }
            lime = null;
        }
    }

    @Override
    public double cameraFps() {
        return lastCameraFps;
    }

    @Override
    public PipelineOutput process(long frameNumber) {
        if (lime == null) {
            return PipelineOutput.EMPTY;
        }

        long tInfer0 = System.nanoTime();
        LLResult result;
        try {
            result = lime.getLatestResult();
        } catch (RuntimeException e) {
            return new PipelineOutput(
                    Collections.emptyList(), 0, null, null, "limelight DC", 0);
        }
        double inferSec = (System.nanoTime() - tInfer0) * 1e-9;

        int dropped = 0;
        long now = System.nanoTime();
        if (lastCaptureNanos > 0) {
            double dt = (now - lastCaptureNanos) * 1e-9;
            double period = 1.0 / Math.max(1.0, POLL_RATE_HZ);
            if (dt > period * 1.5) {
                dropped = Math.max(0, (int) Math.round(dt / period) - 1);
            }
            if (dt > 1e-4) {
                lastCameraFps = 0.85 * lastCameraFps + 0.15 * (1.0 / dt);
            }
        }
        lastCaptureNanos = now;

        if (result == null || !result.isValid()) {
            return new PipelineOutput(Collections.emptyList(), inferSec, null, null, "", dropped);
        }

        List<LLResultTypes.DetectorResult> raw = result.getDetectorResults();
        scratch.clear();
        if (raw != null) {
            for (int i = 0; i < raw.size(); i++) {
                LLResultTypes.DetectorResult d = raw.get(i);
                if (d.getTargetArea() < Limelight.MIN_AREA) {
                    continue;
                }
                if (d.getConfidence() < Limelight.MIN_CONFIDENCE) {
                    continue;
                }
                String name = d.getClassName() == null ? "" : d.getClassName();
                if (Limelight.CLASS_FILTER != null
                        && !Limelight.CLASS_FILTER.isEmpty()
                        && !name.toLowerCase().contains(Limelight.CLASS_FILTER.toLowerCase())) {
                    continue;
                }
                VisionPose robotRel = estimateRobotRelative(d.getTargetXDegrees(), d.getTargetYDegrees());
                scratch.add(new Detection(
                        Integer.toString(i),
                        name,
                        d.getConfidence(),
                        d.getTargetXDegrees(),
                        d.getTargetYDegrees(),
                        d.getTargetArea(),
                        robotRel,
                        null));
            }
        }

        List<Detection> published = scratch.isEmpty()
                ? Collections.emptyList()
                : Collections.unmodifiableList(new ArrayList<>(scratch));

        VisionPose bestRel = null;
        if (!published.isEmpty()) {
            Detection best = published.get(0);
            for (Detection d : published) {
                if (d.area > best.area) {
                    best = d;
                }
            }
            bestRel = best.robotRelative;
        }

        return new PipelineOutput(published, inferSec, bestRel, null, "", dropped);
    }

    public static VisionPose estimateRobotRelative(double txDeg, double tyDeg) {
        double range = estimateRangeInches(tyDeg);
        if (Double.isNaN(range)) {
            return null;
        }
        range = Math.max(Limelight.MIN_RANGE_IN, Math.min(Limelight.MAX_RANGE_IN, range));
        double txRad = Math.toRadians(txDeg);
        double forwardFromCamera = range * Math.cos(txRad);
        double rightFromCamera = range * Math.sin(txRad);
        double forward = Limelight.CAMERA_FORWARD_IN + forwardFromCamera;
        double left = Limelight.CAMERA_LEFT_IN - rightFromCamera;
        return new VisionPose(forward, left, 0);
    }

    public static double estimateRangeInches(double tyDeg) {
        double angleDownRad = Math.toRadians(-(Limelight.CAMERA_PITCH_DEG + tyDeg));
        if (angleDownRad <= 1e-3) {
            return Double.NaN;
        }
        return Limelight.CAMERA_HEIGHT_IN / Math.tan(angleDownRad);
    }
}
