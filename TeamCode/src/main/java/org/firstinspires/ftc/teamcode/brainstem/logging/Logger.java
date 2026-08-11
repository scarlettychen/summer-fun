package org.firstinspires.ftc.teamcode.brainstem.logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.vision.Detection;
import org.firstinspires.ftc.teamcode.brainstem.vision.VisionPose;
import org.firstinspires.ftc.teamcode.brainstem.vision.VisionResult;

import com.pedropathing.geometry.Pose;

import java.io.File;
import java.util.ArrayList;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

public final class Logger {

    public static final String LOG_DIR = "/sdcard/FIRST/logs/";

    private static final int POOL_SIZE = 256;
    private static final int BATCH_MAX = 32;
    private static final long WRITER_POLL_MS = 25;
    private static final long FLUSH_INTERVAL_MS = 250;

    private final ArrayBlockingQueue<LogEntry> free = new ArrayBlockingQueue<>(POOL_SIZE);
    private final ArrayBlockingQueue<LogEntry> filled = new ArrayBlockingQueue<>(POOL_SIZE);
    private final ArrayList<String> extraNames = new ArrayList<>(8);

    private final AtomicBoolean running = new AtomicBoolean(false);
    private final AtomicLong droppedSamples = new AtomicLong(0);
    private final AtomicLong writtenSamples = new AtomicLong(0);

    private Thread writerThread;
    private volatile File logFile;
    private volatile String header = LogEntry.CSV_HEADER;

    public Logger() {
        for (int i = 0; i < POOL_SIZE; i++) {
            free.offer(new LogEntry());
        }
    }

    public int registerExtra(String columnName) {
        if (running.get()) {
            throw new IllegalStateException("registerExtra only before start()");
        }
        if (extraNames.size() >= LogEntry.MAX_EXTRA) {
            throw new IllegalStateException("MAX_EXTRA=" + LogEntry.MAX_EXTRA);
        }
        extraNames.add(sanitize(columnName));
        return extraNames.size() - 1;
    }

    public void start(String opModeName) {
        if (!running.compareAndSet(false, true)) {
            return;
        }

        StringBuilder h = new StringBuilder(LogEntry.CSV_HEADER);
        for (int i = 0; i < extraNames.size(); i++) {
            h.append(',').append(extraNames.get(i));
        }
        header = h.toString();

        for (LogEntry e : free) {
            e.extraCount = extraNames.size();
        }

        for (LogEntry e : filled) {
            e.extraCount = extraNames.size();
        }

        String safe = sanitize(opModeName == null ? "OpMode" : opModeName);
        logFile = new File(LOG_DIR, safe + "_" + System.currentTimeMillis() + ".csv");

        writerThread = new Thread(this::writerLoop, "BrainSTEM-Logger");
        writerThread.setDaemon(true);
        writerThread.setPriority(Thread.NORM_PRIORITY - 1);
        writerThread.start();
    }

    public void update(BrainSTEMRobot robot, double loopTimeMs) {
        if (!running.get() || robot == null) {
            return;
        }
        LogEntry e = borrow();
        if (e == null) {
            droppedSamples.incrementAndGet();
            return;
        }
        try {
            fillFromRobot(e, robot, loopTimeMs);
            commit(e);
        } catch (RuntimeException ex) {

            recycle(e);
        }
    }

    public void update(LogEntry entry) {
        if (entry == null) {
            return;
        }
        if (!running.get()) {
            recycle(entry);
            return;
        }
        commit(entry);
    }

    public LogEntry borrow() {
        LogEntry e = free.poll();
        if (e == null) {
            return null;
        }
        e.reset();
        e.extraCount = extraNames.size();
        return e;
    }

    public void commit(LogEntry entry) {
        if (entry == null) {
            return;
        }
        if (!running.get() || !filled.offer(entry)) {
            droppedSamples.incrementAndGet();
            recycle(entry);
        }
    }

    public void stop() {
        if (!running.compareAndSet(true, false)) {
            return;
        }
        Thread t = writerThread;
        writerThread = null;
        if (t != null) {
            t.interrupt();
            try {
                t.join(1500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public File getLogFile() {
        return logFile;
    }

    public long getDroppedSamples() {
        return droppedSamples.get();
    }

    public long getWrittenSamples() {
        return writtenSamples.get();
    }

    public boolean isRunning() {
        return running.get();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("log", running.get() ? "ON" : "off");
        telemetry.addData("log written", writtenSamples.get());
        telemetry.addData("log dropped", droppedSamples.get());
        if (logFile != null) {
            telemetry.addData("log file", logFile.getName());
        }
    }

    public static void fillFromRobot(LogEntry e, BrainSTEMRobot robot, double loopTimeMs) {
        e.timestampMs = System.currentTimeMillis();
        e.loopTimeMs = loopTimeMs;

        double[] field = robot.getFieldPose();
        e.xIn = field[0];
        e.yIn = field[1];
        e.headingDeg = field[2];

        Pose vel = robot.pinpoint.getVelocity();

        e.velocityInPerSec = Math.hypot(vel.getX(), vel.getY());

        e.batteryV = robot.getBatteryVoltage();

        robot.drive.copyDriveLog(e);

        e.liftPos = robot.lift.getPosition();
        e.liftTarget = robot.lift.getTargetPosition();

        VisionResult vision = robot.vision.getLatestResult();
        e.visionHasTarget = vision.hasTarget ? 1.0 : 0.0;
        e.visionLatencyMs = vision.processingLatencySec * 1000.0;
        Detection best = vision.bestDetection();
        if (best != null) {
            e.visionTx = best.txDegrees;
            e.visionTy = best.tyDegrees;
            e.visionConf = best.confidence;
        }

        e.odoX = field[0];
        e.odoY = field[1];
        e.odoH = field[2];

        VisionPose vPose = vision.robotRelativePose;
        if (vPose != null) {
            e.visionPoseX = vPose.xInches;
            e.visionPoseY = vPose.yInches;
            e.visionPoseH = vPose.headingDegrees;
        }

        e.fusedX = e.odoX;
        e.fusedY = e.odoY;
        e.fusedH = e.odoH;
    }

    private void writerLoop() {
        CsvLogWriter csv = null;
        ArrayList<LogEntry> batch = new ArrayList<>(BATCH_MAX);
        long lastFlushMs = System.currentTimeMillis();
        try {
            csv = new CsvLogWriter(logFile, header);
            while (running.get() || !filled.isEmpty()) {
                batch.clear();
                LogEntry first = filled.poll(WRITER_POLL_MS, TimeUnit.MILLISECONDS);
                if (first != null) {
                    batch.add(first);
                    filled.drainTo(batch, BATCH_MAX - 1);
                }
                if (!batch.isEmpty()) {
                    csv.writeAll(batch);
                    writtenSamples.addAndGet(batch.size());
                    for (int i = 0; i < batch.size(); i++) {
                        recycle(batch.get(i));
                    }
                }
                long now = System.currentTimeMillis();
                if (now - lastFlushMs >= FLUSH_INTERVAL_MS) {
                    csv.flush();
                    lastFlushMs = now;
                }
            }
            csv.flush();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (Exception e) {

        } finally {
            if (csv != null) {
                try {
                    csv.close();
                } catch (Exception ignored) {
                }
            }

            LogEntry leftover;
            while ((leftover = filled.poll()) != null) {
                recycle(leftover);
            }
        }
    }

    private void recycle(LogEntry e) {
        if (e == null) {
            return;
        }
        e.reset();
        e.extraCount = extraNames.size();
        free.offer(e);
    }

    private static String sanitize(String raw) {
        String s = raw == null ? "col" : raw.trim().replaceAll("[^A-Za-z0-9_\\-]+", "_");
        return s.isEmpty() ? "col" : s;
    }
}
