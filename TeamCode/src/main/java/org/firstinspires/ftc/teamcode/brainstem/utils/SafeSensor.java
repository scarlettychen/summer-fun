package org.firstinspires.ftc.teamcode.brainstem.utils;

import java.util.function.Supplier;

public class SafeSensor<T> {

    private final String name;
    private final Supplier<T> reader;

    private T lastGoodValue;
    private boolean healthy = true;
    private int consecutiveFailures = 0;
    private long lastFailureTimestamp = 0;

    public SafeSensor(String name, Supplier<T> reader, T fallback) {
        this.name = name;
        this.reader = reader;
        this.lastGoodValue = fallback;
    }

    public T read() {
        try {
            T value = reader.get();
            if (value == null) {
                throw new NullPointerException(name + " returned null");
            }
            lastGoodValue = value;
            healthy = true;
            consecutiveFailures = 0;
        } catch (Exception e) {
            consecutiveFailures++;
            healthy = false;
            lastFailureTimestamp = System.currentTimeMillis();

        }
        return lastGoodValue;
    }

    public boolean isHealthy() {
        return healthy;
    }

    public int getConsecutiveFailures() {
        return consecutiveFailures;
    }

    public String getName() {
        return name;
    }

    public long getLastFailureTimestamp() {
        return lastFailureTimestamp;
    }
}
