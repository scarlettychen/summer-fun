package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shared runtime information for a single {@link VerifiedCommand} run: when
 * it started, and which attempt this is. {@link Verifier}, {@link RecoveryAction},
 * and {@link RetryPolicy} all read the same instance instead of reaching back
 * into VerifiedCommand directly — that's what keeps those three decoupled
 * from each other.
 *
 * Read-only for everyone except {@link VerifiedCommand}, which owns writing
 * to it (advancing the attempt counter on retry). Whatever subsystem an
 * Action/Verifier/RecoveryAction acts on is already in scope where they're
 * built (they're normally closures over e.g. a {@code FourBarLinkage}
 * reference), so it isn't duplicated here.
 */
public final class ExecutionContext {

    private final ElapsedTime timer = new ElapsedTime();
    private int attemptNumber = 1;

    ExecutionContext() {}

    /** Time since the run started (attempt 1), not since the last retry. */
    public long elapsedMillis() {
        return (long) timer.milliseconds();
    }

    /** Current attempt number — 1 on the first try, incremented on every retry. */
    public int attemptNumber() {
        return attemptNumber;
    }

    /** Internal — only {@link VerifiedCommand} calls this, right before a retry. */
    void advanceAttempt() {
        attemptNumber++;
    }
}
