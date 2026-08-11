package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

/**
 * Records what happened when an attempt failed — the audit trail for the
 * whole system. {@link VerifiedCommand} calls this once per {@link
 * FailureContext} (i.e. once per failed attempt, not once per run), so a run
 * that fails twice then succeeds produces two log calls.
 *
 * Suggested method:
 * - log(FailureContext failureContext) — write it wherever makes sense:
 *   telemetry.addLine(...) for driver-station visibility during a match, or
 *   pipe it into the existing async CSV logger (brainstem/logging/Logger +
 *   LogEntry) if you want failures on record after the match. Don't do
 *   anything blocking here — same rule as everywhere else in this codebase,
 *   this runs on the main robot loop.
 *
 * Keeping this as its own interface (instead of just calling
 * System.out.println inline in VerifiedCommand) means you can swap a
 * telemetry-only logger for a CSV one later without touching the state
 * machine, and you can have zero loggers (no-op) for lightweight commands.
 */
public interface FailureLogger {

    // Called once per failed attempt (not once per run) — write it to
    // telemetry, the async CSV logger, or both. Never block here.
    void log(FailureContext failureContext);


}
