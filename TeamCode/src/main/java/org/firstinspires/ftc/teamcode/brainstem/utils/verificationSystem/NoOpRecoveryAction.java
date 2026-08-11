package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

/**
 * A {@link RecoveryAction} that does nothing and finishes immediately. Pass
 * {@link #INSTANCE} to {@code VerifiedCommand} (or just {@code null} — both
 * mean the same thing) when a failed attempt should simply retry with no
 * corrective step in between.
 */
public final class NoOpRecoveryAction implements RecoveryAction {

    public static final NoOpRecoveryAction INSTANCE = new NoOpRecoveryAction();

    @Override
    public void begin(ExecutionContext context) {
        // Nothing to do.
    }

    @Override
    public void update() {
        // Nothing to do.
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
