package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

/**
 * The final outcome of a {@link VerifiedCommand} run, handed back to whoever
 * scheduled it once the command is {@code isFinished()}. This is the one
 * object the *caller* (an auto step, a teleop button handler, etc.) actually
 * looks at — everything else in this package is internal machinery the
 * caller never sees.
 *
 * Immutable; build one via {@link #success} / {@link #failure} rather than a
 * public constructor.
 */
public final class ActionResult {

    private final boolean success;
    private final int totalAttempts;
    private final long elapsedMillis;
    private final FailureContext lastFailure;

    private ActionResult(boolean success, int totalAttempts, long elapsedMillis, FailureContext lastFailure) {
        this.success = success;
        this.totalAttempts = totalAttempts;
        this.elapsedMillis = elapsedMillis;
        this.lastFailure = lastFailure;
    }

    /** True if the run ultimately succeeded (first try or after retries). */
    public boolean isSuccess() {
        return success;
    }

    /** Total attempts made — 1 if it succeeded on the first try. */
    public int getTotalAttempts() {
        return totalAttempts;
    }

    /** Total elapsed time for the whole run, from first attempt to this result. */
    public long getElapsedMillis() {
        return elapsedMillis;
    }

    /** The last failure, if the run ended in failure; null on success. */
    public FailureContext getLastFailure() {
        return lastFailure;
    }

    public static ActionResult success(int totalAttempts, long elapsedMillis) {
        return new ActionResult(true, totalAttempts, elapsedMillis, null);
    }

    public static ActionResult failure(int totalAttempts, long elapsedMillis, FailureContext lastFailure) {
        return new ActionResult(false, totalAttempts, elapsedMillis, lastFailure);
    }

    @Override
    public String toString() {
        return success
                ? String.format("SUCCESS after %d attempt(s), %dms", totalAttempts, elapsedMillis)
                : String.format(
                        "FAILURE after %d attempt(s), %dms: %s", totalAttempts, elapsedMillis, lastFailure);
    }
}
