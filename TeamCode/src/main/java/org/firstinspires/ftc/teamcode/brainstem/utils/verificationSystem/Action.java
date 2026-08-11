package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

/**
 * The framework-neutral "thing that runs across ticks" contract. This is
 * what {@link VerifiedCommand} wraps and verifies — deliberately NOT tied to
 * Ivy's {@code Command} or any other scheduler's interface, so a
 * VerifiedCommand can verify literally anything with a start/tick/finished
 * shape: an instant setter, a whole path-following segment, or a command
 * from a different framework entirely.
 *
 * This shape is intentionally the common denominator of the two command
 * styles this SDK cares about:
 *
 * | Action (this)              | Ivy {@code Command}         | FTCLib {@code Command}        |
 * |-----------------------------|------------------------------|--------------------------------|
 * | {@code start()}             | {@code start()}              | {@code initialize()}          |
 * | {@code execute()}           | {@code execute()}             | {@code execute()}              |
 * | {@code isFinished()}        | {@code done()}                | {@code isFinished()}           |
 * | {@code end(interrupted)}    | {@code end(EndCondition)}     | {@code end(interrupted)}       |
 *
 * Adapters convert each direction:
 * - {@link IvyCommandAction} — wraps an existing Ivy {@code Command} so it
 *   can be passed into a {@code VerifiedCommand} as the action to verify.
 * - {@link IvyCommandAdapter} — wraps any {@code Action} (typically a
 *   {@code VerifiedCommand}) so it can be scheduled directly with Ivy's
 *   {@code Scheduler}/{@code Groups}.
 *
 * Write the equivalent pair (e.g. {@code FtcLibCommandAction} /
 * {@code FtcLibCommandAdapter}) if/when this SDK adds FTCLib — the method
 * names above were chosen to make that a near 1:1 mapping.
 *
 * Suggested methods:
 * - start() — called once when the action begins (or begins again, on retry).
 * - execute() — called every tick while running.
 * - isFinished() — true once the action itself is done running. For an
 *   instant action (e.g. "set a target state") this can return true the
 *   tick after start(). This says nothing about whether the action
 *   *worked* — that's still the {@link Verifier}'s job.
 * - end(boolean interrupted) — cleanup hook, called once after isFinished()
 *   is true, or early if VerifiedCommand is cancelled mid-flight
 *   (interrupted = true in that case).
 */
public interface Action {

    // Called once when the action begins (or begins again, on retry).
    void start();

    // Called every tick while the action is running.
    void execute();

    // True once the action itself is done running — says nothing about
    // whether it *worked*, that's the Verifier's job.
    boolean isFinished();

    // Cleanup hook, called once after isFinished() is true, or early
    // (interrupted = true) if the action is cancelled mid-flight.
    void end(boolean interrupted);
}
