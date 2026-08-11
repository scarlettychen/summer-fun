package org.firstinspires.ftc.teamcode.brainstem.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public final class VisionSubsystem {

    private static final double EWMA_ALPHA = 0.15;

    private final VisionPipeline pipeline;
    private final AtomicReference<VisionResult> latest =
            new AtomicReference<>(VisionResult.EMPTY);
    private final AtomicBoolean running = new AtomicBoolean(false);
    private final AtomicBoolean threadAlive = new AtomicBoolean(false);
    private final AtomicLong droppedFrames = new AtomicLong(0);
    private final AtomicLong processedFrames = new AtomicLong(0);

    private volatile double ewmaFps;
    private volatile double ewmaLatencySec;
    private volatile double ewmaInferenceSec;
    private volatile double cameraFps;

    private Thread worker;
    private final Object lifecycleLock = new Object();

    public VisionSubsystem(VisionPipeline pipeline) {
        if (pipeline == null) {
            throw new IllegalArgumentException("pipeline required");
        }
        this.pipeline = pipeline;
    }

    public void start() {
        synchronized (lifecycleLock) {
            if (running.get()) {
                return;
            }
            running.set(true);
            worker = new Thread(this::runLoop, "BrainSTEM-Vision");
            worker.setDaemon(true);
            worker.setPriority(Thread.NORM_PRIORITY + 1);
            worker.start();
        }
    }

    public void stop() {
        Thread toJoin;
        synchronized (lifecycleLock) {
            if (!running.getAndSet(false)) {
                return;
            }
            toJoin = worker;
            worker = null;
        }
        if (toJoin != null) {
            toJoin.interrupt();
            try {
                toJoin.join(750);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        threadAlive.set(false);
    }

    public VisionResult getLatestResult() {
        VisionResult r = latest.get();
        return r == null ? VisionResult.EMPTY : r;
    }

    public boolean hasTarget() {
        return getLatestResult().hasTarget;
    }

    public VisionPose getEstimatedPose() {
        VisionPose p = getLatestResult().robotRelativePose;
        return p == null ? VisionPose.ZERO : p;
    }

    public List<Detection> getDetections() {
        return getLatestResult().detections;
    }

    public VisionStats getStats() {
        VisionResult r = getLatestResult();
        return new VisionStats(
                ewmaFps,
                ewmaLatencySec,
                ewmaInferenceSec,
                cameraFps,
                droppedFrames.get(),
                processedFrames.get(),
                r.timestampNanos,
                threadAlive.get(),
                r.hasTarget,
                r.bestConfidence,
                pipeline.name());
    }

    public void addTelemetry(Telemetry telemetry) {
        getStats().addTelemetry(telemetry, "vision");
        VisionResult r = getLatestResult();
        if (r.hasTarget) {
            Detection best = r.bestDetection();
            if (best != null) {
                telemetry.addData("vision best", "%s conf=%.2f tx=%.1f",
                        best.className, best.confidence, best.txDegrees);
            }
        }
        if (!r.debug.isEmpty()) {
            telemetry.addData("vision dbg", r.debug);
        }
    }

    public VisionPipeline getPipeline() {
        return pipeline;
    }

    public boolean isRunning() {
        return running.get() && threadAlive.get();
    }

    private void runLoop() {
        threadAlive.set(true);
        try {
            pipeline.open();
        } catch (RuntimeException e) {
            running.set(false);
            threadAlive.set(false);
            latest.set(new VisionResult(
                    System.nanoTime(), 0, 0, 0,
                    Collections.emptyList(), 0, false, null, null,
                    pipeline.name(), "open failed: " + e.getMessage()));
            return;
        }

        long frameNumber = 0;
        long lastLoopNanos = System.nanoTime();

        try {
            while (running.get() && !Thread.currentThread().isInterrupted()) {
                long t0 = System.nanoTime();
                frameNumber++;

                VisionPipeline.PipelineOutput out;
                try {
                    out = pipeline.process(frameNumber);
                } catch (RuntimeException e) {
                    out = new VisionPipeline.PipelineOutput(
                            Collections.emptyList(), 0, null, null,
                            "process error: " + e.getMessage(), 1);
                }
                if (out == null) {
                    out = VisionPipeline.PipelineOutput.EMPTY;
                }

                long t1 = System.nanoTime();
                double latencySec = (t1 - t0) * 1e-9;
                double dtSec = (t0 - lastLoopNanos) * 1e-9;
                lastLoopNanos = t0;

                if (out.droppedWhileProcessing > 0) {
                    droppedFrames.addAndGet(out.droppedWhileProcessing);
                }

                Detection best = null;
                double bestConf = 0;
                for (Detection d : out.detections) {
                    if (best == null
                            || d.area > best.area
                            || (d.area == best.area && d.confidence > best.confidence)) {
                        best = d;
                        bestConf = d.confidence;
                    }
                }
                boolean hasTarget = best != null;
                VisionPose robotRel = out.robotRelativePose;
                VisionPose targetRel = out.targetRelativePose;
                if (robotRel == null && best != null) {
                    robotRel = best.robotRelative;
                }
                if (targetRel == null && best != null) {
                    targetRel = best.targetPose;
                }

                VisionResult published = new VisionResult(
                        t1,
                        frameNumber,
                        latencySec,
                        out.inferenceTimeSec > 0 ? out.inferenceTimeSec : latencySec,
                        out.detections,
                        bestConf,
                        hasTarget,
                        robotRel,
                        targetRel,
                        pipeline.name(),
                        out.debug);
                latest.set(published);
                processedFrames.incrementAndGet();

                if (dtSec > 1e-4) {
                    double instFps = 1.0 / dtSec;
                    ewmaFps = ewma(ewmaFps, instFps);
                }
                ewmaLatencySec = ewma(ewmaLatencySec, latencySec);
                ewmaInferenceSec = ewma(ewmaInferenceSec, published.inferenceTimeSec);
                double cam = pipeline.cameraFps();
                if (cam > 0) {
                    cameraFps = ewma(cameraFps, cam);
                }

                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        } finally {
            try {
                pipeline.close();
            } catch (RuntimeException ignored) {

            }
            threadAlive.set(false);
            running.set(false);
        }
    }

    private static double ewma(double prev, double sample) {
        if (prev <= 0) {
            return sample;
        }
        return prev + EWMA_ALPHA * (sample - prev);
    }
}
