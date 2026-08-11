package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

/**
 * Performs one corrective action in response to a failure — e.g. reverse
 * the intake for 300ms to clear a jam, nudge the lift with extra power, or
 * re-zero an encoder. This is deliberately its own small lifecycle rather
 * than a plain Runnable, because most real recoveries take more than one
 * tick (you can't "un-jam" instantly).
 *
 * {@link VerifiedCommand} drives this the same way it drives the main action:
 * call {@link #begin} once per failure, then {@link #update} every tick,
 * then move on once {@link #isFinished} is true — at which point
 * VerifiedCommand asks the {@link RetryPolicy} whether to retry the original
 * action. There's exactly one RecoveryAction per VerifiedCommand (or none —
 * pass null); if different attempts should recover differently, branch on
 * {@code context.attemptNumber()} inside {@link #begin} itself.
 *
 * Suggested methods:
 * - begin(ExecutionContext context) — called once per failed attempt, right
 *   before recovery starts. Check context.attemptNumber() here if different
 *   attempts should recover differently.
 * - update() — called every tick while recovering (run motors, check timers).
 * - isFinished() — return true once the corrective action is done running
 *   (this does NOT mean the original problem is fixed — that's re-checked by
 *   the Verifier on the next attempt).
 *
 * A RecoveryAction that does nothing (isFinished() true immediately) is a
 * valid "no-op recovery" — see {@link NoOpRecoveryAction} — useful when you
 * want a plain retry with no corrective step in between.
 *
 * Deliberately not the same shape as {@link Action}: begin() takes the
 * {@link ExecutionContext} directly because recovery is always reacting to a
 * specific failure, whereas Action is meant to be usable standalone (e.g.
 * wrapping a bare Ivy Command that knows nothing about this package).
 */
public interface RecoveryAction {

    // Called once when recovery starts — grab whatever Component reference
    // you need from the context here.
    void begin(ExecutionContext context);

    // Called every tick while recovering (run motors, check timers).
    void update();

    // True once the corrective action is done running — does NOT mean the
    // original problem is fixed, that's re-checked by the Verifier on retry.
    boolean isFinished();
}
