package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

public interface VerificationResult {
    enum VerificationType {
        SUCCESS,
        FAILED,
        UNCERTAIN
    }

    // What a Verifier concluded on this tick.
    VerificationType getType();
}
