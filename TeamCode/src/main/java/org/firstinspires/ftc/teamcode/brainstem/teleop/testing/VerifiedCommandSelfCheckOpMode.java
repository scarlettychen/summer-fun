package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.utils.verificationSystem.VerifiedCommandExamples;

/**
 * Runs the {@link VerifiedCommandExamples} self-check (no hardware needed):
 * immediate success, uncertain-then-success, fail/recover/retry/succeed, and
 * exhausting retries. Confirms the verify → recover → retry state machine
 * behaves as designed before it's ever wired to a real subsystem.
 */
@TeleOp(name = "Verified Command Self-Check", group = "SysId")
public class VerifiedCommandSelfCheckOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Verified Command self-check");
        telemetry.addLine("Press start to run assertions");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        try {
            VerifiedCommandExamples.runSelfCheck();
            telemetry.addLine("PASS — success / uncertain / recover-retry / exhausted-retry cases OK");
        } catch (AssertionError e) {
            telemetry.addLine("FAIL");
            telemetry.addLine(e.getMessage());
        }
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
