package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

/**
 * The obvious concrete {@link VerificationResult}: just a {@code VerificationType},
 * nothing else. Use the static constants below in a {@link Verifier} instead
 * of writing a one-off implementation every time:
 *
 * <pre>{@code
 * return lift.atTarget() ? SimpleVerificationResult.SUCCESS : SimpleVerificationResult.UNCERTAIN;
 * }</pre>
 */
public final class SimpleVerificationResult implements VerificationResult {

    public static final VerificationResult SUCCESS = new SimpleVerificationResult(VerificationType.SUCCESS);
    public static final VerificationResult FAILED = new SimpleVerificationResult(VerificationType.FAILED);
    public static final VerificationResult UNCERTAIN = new SimpleVerificationResult(VerificationType.UNCERTAIN);

    private final VerificationType type;

    public SimpleVerificationResult(VerificationType type) {
        this.type = type;
    }

    @Override
    public VerificationType getType() {
        return type;
    }

    @Override
    public String toString() {
        return type.name();
    }
}
