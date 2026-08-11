package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.follower.PathPlannerImportExamples;
import org.firstinspires.ftc.teamcode.brainstem.follower.VelocityConstraintExamples;

/**
 * Runs both follower self-checks (no field time needed): dynamic velocity limiting
 * math and planner-JSON parsing (units, corner/center origin, cubic/quadratic handles).
 */
@TeleOp(name = "Path Follower Self-Check", group = "SysId")
public class PathFollowerSelfCheckOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Path Follower self-check");
        telemetry.addLine("Press start to run assertions");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        report("VelocityConstraint", VelocityConstraintExamples::runSelfCheck);
        report("PathPlannerImport", PathPlannerImportExamples::runSelfCheck);
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }

    private void report(String label, Runnable check) {
        try {
            check.run();
            telemetry.addData(label, "PASS");
        } catch (AssertionError e) {
            telemetry.addData(label, "FAIL — " + e.getMessage());
        }
    }
}
