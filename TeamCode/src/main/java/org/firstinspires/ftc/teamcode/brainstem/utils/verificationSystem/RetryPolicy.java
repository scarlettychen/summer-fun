package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

/**
 * Decides whether {@link VerifiedCommand} should re-run the original action
 * after a failure (and its recovery, if any) has completed. Called last in
 * the failure chain — after {@link RecoveryAction} has already run — so it
 * can factor in "did recovery even finish ok."
 *
 * Typical logic: cap attempts, e.g. {@code failureContext.getExecutionContext()
 * .attemptNumber() < maxAttempts} — see {@link MaxAttemptsRetryPolicy}.
 *
 * If the next attempt needs a moment before it starts (e.g. letting a PID
 * settle), build that delay into the {@link RecoveryAction} instead — e.g. a
 * recovery that does nothing but wait a few ticks before {@code isFinished()}
 * returns true. RetryPolicy itself is a single yes/no decision.
 */
public interface RetryPolicy {

    // False means give up — VerifiedCommand moves to ActionResult.failure(...).
    boolean shouldRetry(FailureContext failureContext);
}
