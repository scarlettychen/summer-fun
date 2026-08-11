package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

/**
 * Checks whether the *physical* outcome of an action actually happened —
 * as opposed to just "the code that commands it ran." This is the whole
 * point of the system: a motor.setPower() call is not proof anything moved.
 *
 * {@link VerifiedCommand} calls {@link #verify} once per tick after it runs
 * the wrapped action, until it gets back something other than UNCERTAIN.
 *
 * Implementations read hardware/sensor state through the {@code Component}
 * referenced by the given {@link ExecutionContext} (e.g. call
 * {@code FourBarLinkage.atTarget()}, or check an encoder position captured
 * before the action ran against the position read now).
 *
 * Return SUCCESS the instant the outcome is confirmed. Return FAILED only
 * when you're sure it did *not* happen (not just "not yet") — e.g. a timeout
 * elapsed, or a sensor reading is physically impossible for success. Return
 * UNCERTAIN for "can't tell yet, check again next tick" (e.g. a PID is still
 * approaching target, or a debounce window hasn't closed).
 *
 * Guardrail to implement yourself: an UNCERTAIN that never resolves will spin
 * forever. Track elapsed time (via ExecutionContext) inside your Verifier and
 * downgrade UNCERTAIN to FAILED past some timeout — don't rely on the caller
 * to catch that for you.
 */
public interface Verifier {

    // Checked once per tick — SUCCESS/FAILED are terminal for this attempt,
    // UNCERTAIN means "check again next tick."
    VerificationResult verify(ExecutionContext context);
}
