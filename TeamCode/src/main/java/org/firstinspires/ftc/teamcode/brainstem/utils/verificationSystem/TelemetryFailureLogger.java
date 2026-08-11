package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The obvious {@link FailureLogger}: one telemetry line per failed attempt,
 * for driver-station visibility during a match. Non-blocking (telemetry
 * writes are just buffered until the next {@code telemetry.update()}), so
 * it's safe to use directly on the robot loop.
 *
 * For a permanent record after the match, pair this with (or swap it for) a
 * logger that writes into {@code brainstem/logging/Logger}.
 */
public final class TelemetryFailureLogger implements FailureLogger {

    private final Telemetry telemetry;
    private final String caption;

    public TelemetryFailureLogger(Telemetry telemetry, String caption) {
        this.telemetry = telemetry;
        this.caption = caption == null || caption.isEmpty() ? "verify" : caption;
    }

    @Override
    public void log(FailureContext failureContext) {
        telemetry.addData(caption, failureContext.toString());
    }
}
