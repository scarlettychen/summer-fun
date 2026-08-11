package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

import java.util.Objects;

/**
 * Runs one action and manages its whole lifecycle: execute → verify →
 * (success | uncertain → verify again | failure → recover → retry-or-stop).
 * This is the orchestrator that ties every other class in this package
 * together — it is the only class that calls into all of Verifier,
 * RecoveryAction, and RetryPolicy.
 *
 * Implements {@link Action} (not Ivy's {@code Command} directly) as its own
 * outer contract. Staying framework-neutral here is what makes this usable
 * from Ivy (via {@link IvyCommandAction}/{@link IvyCommandAdapter}) or any
 * other command framework, with zero changes to this class.
 *
 * See {@code docs/VERIFIED_COMMANDS.md} for the full flow diagram and a
 * worked example.
 */
public final class VerifiedCommand implements Action {

    private enum State {
        EXECUTING,
        VERIFYING,
        RECOVERING,
        DONE
    }

    private final Action action;
    private final Verifier verifier;
    private final RetryPolicy retryPolicy;
    private final RecoveryAction recovery;
    private final FailureLogger failureLogger;

    private State state = State.DONE;
    private ExecutionContext context;
    private FailureContext pendingFailure;
    private ActionResult result;

    public VerifiedCommand(Action action, Verifier verifier, RetryPolicy retryPolicy) {
        this(action, verifier, retryPolicy, null, null);
    }

    public VerifiedCommand(
            Action action,
            Verifier verifier,
            RetryPolicy retryPolicy,
            RecoveryAction recovery,
            FailureLogger failureLogger) {
        this.action = Objects.requireNonNull(action, "action");
        this.verifier = Objects.requireNonNull(verifier, "verifier");
        this.retryPolicy = Objects.requireNonNull(retryPolicy, "retryPolicy");
        this.recovery = recovery;
        this.failureLogger = failureLogger;
    }

    public static Builder builder() {
        return new Builder();
    }

    @Override
    public void start() {
        context = new ExecutionContext();
        pendingFailure = null;
        result = null;
        state = State.EXECUTING;
        action.start();
    }

    @Override
    public void execute() {
        switch (state) {
            case EXECUTING:
                tickExecuting();
                break;
            case VERIFYING:
                tickVerifying();
                break;
            case RECOVERING:
                tickRecovering();
                break;
            case DONE:
            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted || state == State.DONE) {
            return;
        }
        if (state == State.EXECUTING) {
            action.end(true);
        }
        if (result == null) {
            result = ActionResult.failure(
                    context != null ? context.attemptNumber() : 0,
                    context != null ? context.elapsedMillis() : 0,
                    pendingFailure);
        }
        state = State.DONE;
    }

    /**
     * Valid once {@link #isFinished()} is true. {@link Action} has no return
     * value, so this is the only way the caller finds out what happened.
     */
    public ActionResult getResult() {
        return result;
    }

    private void tickExecuting() {
        action.execute();
        if (action.isFinished()) {
            action.end(false);
            state = State.VERIFYING;
        }
    }

    private void tickVerifying() {
        VerificationResult verification = verifier.verify(context);
        switch (verification.getType()) {
            case SUCCESS:
                result = ActionResult.success(context.attemptNumber(), context.elapsedMillis());
                state = State.DONE;
                break;
            case UNCERTAIN:
                break;
            case FAILED:
                handleFailure(verification);
                break;
        }
    }

    private void handleFailure(VerificationResult verification) {
        FailureContext failure = new FailureContext(verification, context);
        pendingFailure = failure;
        if (failureLogger != null) {
            failureLogger.log(failure);
        }

        if (recovery != null) {
            recovery.begin(context);
            state = State.RECOVERING;
        } else {
            decideRetry(failure);
        }
    }

    private void tickRecovering() {
        recovery.update();
        if (recovery.isFinished()) {
            decideRetry(pendingFailure);
        }
    }

    private void decideRetry(FailureContext failure) {
        if (retryPolicy.shouldRetry(failure)) {
            context.advanceAttempt();
            state = State.EXECUTING;
            action.start();
        } else {
            result = ActionResult.failure(context.attemptNumber(), context.elapsedMillis(), failure);
            state = State.DONE;
        }
    }

    /** Fluent construction, matching the feel of Ivy's own {@code CommandBuilder}. */
    public static final class Builder {
        private Action action;
        private Verifier verifier;
        private RetryPolicy retryPolicy;
        private RecoveryAction recovery;
        private FailureLogger failureLogger;

        private Builder() {}

        public Builder action(Action action) {
            this.action = action;
            return this;
        }

        public Builder verifier(Verifier verifier) {
            this.verifier = verifier;
            return this;
        }

        public Builder retryPolicy(RetryPolicy retryPolicy) {
            this.retryPolicy = retryPolicy;
            return this;
        }

        public Builder recovery(RecoveryAction recovery) {
            this.recovery = recovery;
            return this;
        }

        public Builder failureLogger(FailureLogger failureLogger) {
            this.failureLogger = failureLogger;
            return this;
        }

        public VerifiedCommand build() {
            return new VerifiedCommand(action, verifier, retryPolicy, recovery, failureLogger);
        }
    }
}
