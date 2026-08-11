package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

/**
 * Describes one specific failure: what happened, on which attempt, and how
 * much time led up to it. Built by {@link VerifiedCommand} the moment a
 * {@link Verifier} returns {@code FAILED}, then handed to {@link RecoveryAction},
 * {@link RetryPolicy}, and {@link FailureLogger} — they all read the same
 * object so their decisions/logs agree on "what failed."
 *
 * Immutable — a new one is built per failed attempt rather than mutating one
 * across attempts. Note this is NOT the same as the final result of the
 * whole VerifiedCommand run: a single run can produce several FailureContexts
 * before either succeeding on a retry or giving up — see {@link ActionResult}
 * for the final outcome.
 */
public final class FailureContext {

    private final VerificationResult verificationResult;
    private final ExecutionContext executionContext;
    private final long timestampMillis;

    public FailureContext(VerificationResult verificationResult, ExecutionContext executionContext) {
        this.verificationResult = verificationResult;
        this.executionContext = executionContext;
        this.timestampMillis = System.currentTimeMillis();
    }

    /** The VerificationResult that triggered this failure. */
    public VerificationResult getVerificationResult() {
        return verificationResult;
    }

    /** The run's ExecutionContext — attempt number and elapsed time. */
    public ExecutionContext getExecutionContext() {
        return executionContext;
    }

    /** When this specific failure happened (millis), separate from the run's start time. */
    public long getTimestampMillis() {
        return timestampMillis;
    }

    @Override
    public String toString() {
        return String.format(
                "attempt %d failed after %dms (%s)",
                executionContext.attemptNumber(), executionContext.elapsedMillis(), verificationResult.getType());
    }


}
