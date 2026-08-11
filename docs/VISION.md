# Asynchronous vision subsystem

`brainstem/vision` runs camera / NN work on a **dedicated background thread**.
The robot loop never waits on vision.

```
VisionSubsystem (thread)
    │  newest frame only (no queue)
    ▼
VisionPipeline  ← Limelight | OpenCV stub | TFLite stub
    │  immutable VisionResult
    ▼
AtomicReference  ←── robot.vision.getLatestResult()  (non-blocking)
```

## API

```java
robot.vision.start();   // already called in BrainSTEMRobot ctor
VisionResult r = robot.vision.getLatestResult();
boolean hit = robot.vision.hasTarget();
VisionPose body = robot.vision.getEstimatedPose();
List<Detection> all = robot.vision.getDetections();
robot.vision.addTelemetry(telemetry);
robot.stopVision();     // OpMode end
```

## Swap backends

```java
// default in BrainSTEMRobot:
new VisionSubsystem(new LimelightPipeline(hardwareMap));

// later:
new VisionSubsystem(new OpenCvVisionPipeline());
new VisionSubsystem(new TensorFlowVisionPipeline("balls.tflite"));
```

Robot chase / tele code stays on `robot.limelight` (facade) or reads `robot.vision` directly.

## Performance rules (enforced by design)

- No frame queue — always process the newest sample
- Publish only immutable `VisionResult` / `Detection` / `VisionPose`
- Prefer `AtomicReference` over locks for the hot path
- Reuse scratch lists / Mats inside the pipeline
- Stats: FPS, latency, inference time, dropped frames, thread alive

See `VisionMonitorOpMode` for a live dashboard.
