package org.firstinspires.ftc.teamcode.brainstem.utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

public class DeathLogger {

    private static final String LOG_DIR = "/sdcard/FIRST/logs/";

    private final BlockingQueue<String> queue = new LinkedBlockingQueue<>();
    private volatile boolean running = false;
    private Thread writerThread;
    private File logFile;

    public void start(String opModeName) {
        new File(LOG_DIR).mkdirs();
        logFile = new File(LOG_DIR + opModeName + "_" + System.currentTimeMillis() + ".txt");
        running = true;

        writerThread = new Thread(() -> {
            try (BufferedWriter writer = new BufferedWriter(new FileWriter(logFile, true))) {
                while (running || !queue.isEmpty()) {
                    String line = queue.poll(200, TimeUnit.MILLISECONDS);
                    if (line != null) {
                        writer.write(line);
                        writer.newLine();
                        writer.flush();

                    }
                }
            } catch (IOException | InterruptedException e) {

            }
        }, "DeathLogger-Writer");
        writerThread.setDaemon(true);
        writerThread.start();
    }

    public void log(double loopTimeMs, double batteryVoltage, String activeMechanisms) {
        if (!running) return;
        String line = String.format("%d,loopMs=%.1f,battery=%.2f,mechanisms=%s",
                System.currentTimeMillis(), loopTimeMs, batteryVoltage, activeMechanisms);
        queue.offer(line);
    }

    public void stop() {
        running = false;
        if (writerThread != null) {
            try {
                writerThread.join(500);
            } catch (InterruptedException ignored) {
            }
        }
    }

    public File getLogFile() {
        return logFile;
    }
}
