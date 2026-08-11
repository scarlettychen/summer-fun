package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

/**
 * The obvious {@link RetryPolicy}: retry up to a fixed number of attempts.
 * Covers most subsystems — write a custom policy only if you need an overall
 * time budget instead of an attempt count.
 */
public final class MaxAttemptsRetryPolicy implements RetryPolicy {

    private final int maxAttempts;

    public MaxAttemptsRetryPolicy(int maxAttempts) {
        this.maxAttempts = maxAttempts;
    }

    @Override
    public boolean shouldRetry(FailureContext failureContext) {
        return failureContext.getExecutionContext().attemptNumber() < maxAttempts;
    }
}
