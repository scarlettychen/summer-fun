package org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem;

/**
 * Self-check for the {@link VerifiedCommand} state machine, using synthetic
 * {@link Action}/{@link Verifier}/{@link RecoveryAction} stand-ins — no
 * hardware required, so this can run on any Driver Station.
 */
public final class VerifiedCommandExamples {

    private VerifiedCommandExamples() {}

    public static void runSelfCheck() {
        immediateSuccess();
        uncertainThenSuccess();
        failThenRetryThenSucceed();
        exhaustsRetriesAndLogs();
    }

    private static void immediateSuccess() {
        ScriptedAction action = new ScriptedAction(1);
        ScriptedVerifier verifier = new ScriptedVerifier(VerificationResult.VerificationType.SUCCESS);
        VerifiedCommand command = new VerifiedCommand(action, verifier, new MaxAttemptsRetryPolicy(3));

        ActionResult result = runToCompletion(command, 10);

        assertTrue("immediate success", result.isSuccess());
        assertTrue("one attempt", result.getTotalAttempts() == 1);
        assertTrue("action started exactly once", action.startCount == 1);
    }

    private static void uncertainThenSuccess() {
        ScriptedAction action = new ScriptedAction(1);
        ScriptedVerifier verifier = new ScriptedVerifier(
                VerificationResult.VerificationType.UNCERTAIN,
                VerificationResult.VerificationType.UNCERTAIN,
                VerificationResult.VerificationType.SUCCESS);
        VerifiedCommand command = new VerifiedCommand(action, verifier, new MaxAttemptsRetryPolicy(3));

        ActionResult result = runToCompletion(command, 10);

        assertTrue("eventually succeeds", result.isSuccess());
        assertTrue("no retries needed", result.getTotalAttempts() == 1);
        assertTrue("action started exactly once despite repeated verify calls", action.startCount == 1);
    }

    private static void failThenRetryThenSucceed() {
        ScriptedAction action = new ScriptedAction(1);
        ScriptedVerifier verifier = new ScriptedVerifier(
                VerificationResult.VerificationType.FAILED, VerificationResult.VerificationType.SUCCESS);
        CountingRecoveryAction recovery = new CountingRecoveryAction(2);
        VerifiedCommand command =
                new VerifiedCommand(action, verifier, new MaxAttemptsRetryPolicy(3), recovery, null);

        ActionResult result = runToCompletion(command, 20);

        assertTrue("succeeds after one retry", result.isSuccess());
        assertTrue("two attempts", result.getTotalAttempts() == 2);
        assertTrue("action restarted for the retry", action.startCount == 2);
        assertTrue("recovery began exactly once", recovery.beginCount == 1);
        assertTrue("recovery ticked to completion before the retry", recovery.updateCount == 2);
    }

    private static void exhaustsRetriesAndLogs() {
        ScriptedAction action = new ScriptedAction(1);
        ScriptedVerifier verifier = new ScriptedVerifier(VerificationResult.VerificationType.FAILED);
        int[] logCount = {0};
        FailureLogger logger = failureContext -> logCount[0]++;
        VerifiedCommand command =
                new VerifiedCommand(action, verifier, new MaxAttemptsRetryPolicy(2), null, logger);

        ActionResult result = runToCompletion(command, 20);

        assertTrue("gives up once max attempts is reached", !result.isSuccess());
        assertTrue("two attempts total", result.getTotalAttempts() == 2);
        assertTrue("logged once per failed attempt", logCount[0] == 2);
    }

    private static ActionResult runToCompletion(VerifiedCommand command, int maxTicks) {
        command.start();
        for (int i = 0; i < maxTicks && !command.isFinished(); i++) {
            command.execute();
        }
        if (!command.isFinished()) {
            throw new AssertionError("VerifiedCommand did not finish within " + maxTicks + " ticks");
        }
        return command.getResult();
    }

    private static void assertTrue(String label, boolean condition) {
        if (!condition) {
            throw new AssertionError("VerifiedCommandExamples: " + label);
        }
    }

    /** Finishes after a fixed number of execute() ticks; counts how many times it was (re)started. */
    private static final class ScriptedAction implements Action {
        private final int ticksToFinish;
        private int remaining;
        int startCount;

        ScriptedAction(int ticksToFinish) {
            this.ticksToFinish = ticksToFinish;
        }

        @Override
        public void start() {
            startCount++;
            remaining = ticksToFinish;
        }

        @Override
        public void execute() {
            if (remaining > 0) {
                remaining--;
            }
        }

        @Override
        public boolean isFinished() {
            return remaining <= 0;
        }

        @Override
        public void end(boolean interrupted) {
            // Nothing to clean up.
        }
    }

    /** Returns a scripted sequence of VerificationTypes, one per call; repeats the last entry once exhausted. */
    private static final class ScriptedVerifier implements Verifier {
        private final VerificationResult.VerificationType[] script;
        private int index;

        ScriptedVerifier(VerificationResult.VerificationType... script) {
            this.script = script;
        }

        @Override
        public VerificationResult verify(ExecutionContext context) {
            VerificationResult.VerificationType type = script[Math.min(index, script.length - 1)];
            index++;
            switch (type) {
                case SUCCESS:
                    return SimpleVerificationResult.SUCCESS;
                case FAILED:
                    return SimpleVerificationResult.FAILED;
                default:
                    return SimpleVerificationResult.UNCERTAIN;
            }
        }
    }

    /** Finishes after a fixed number of update() ticks; counts begin()/update() calls. */
    private static final class CountingRecoveryAction implements RecoveryAction {
        private final int ticksToFinish;
        private int remaining;
        int beginCount;
        int updateCount;

        CountingRecoveryAction(int ticksToFinish) {
            this.ticksToFinish = ticksToFinish;
        }

        @Override
        public void begin(ExecutionContext context) {
            beginCount++;
            remaining = ticksToFinish;
        }

        @Override
        public void update() {
            updateCount++;
            if (remaining > 0) {
                remaining--;
            }
        }

        @Override
        public boolean isFinished() {
            return remaining <= 0;
        }
    }
}
