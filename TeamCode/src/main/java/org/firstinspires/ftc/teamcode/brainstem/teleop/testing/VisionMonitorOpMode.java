package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.vision.Detection;
import org.firstinspires.ftc.teamcode.brainstem.vision.VisionResult;

@TeleOp(name = "Vision Monitor", group = "Test")
public class VisionMonitorOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this);

        telemetry.addLine("Vision thread running independently");
        telemetry.addLine("Robot loop only calls getLatestResult()");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.update();

            VisionResult result = robot.vision.getLatestResult();
            robot.vision.addTelemetry(telemetry);
            telemetry.addData("hasTarget", result.hasTarget);
            telemetry.addData("frame", result.frameNumber);
            telemetry.addData("latency ms", "%.1f", result.processingLatencySec * 1000.0);
            Detection best = result.bestDetection();
            if (best != null) {
                telemetry.addData("best", "%s conf=%.2f tx=%.1f ty=%.1f",
                        best.className, best.confidence, best.txDegrees, best.tyDegrees);
            }
            if (result.robotRelativePose != null) {
                telemetry.addData("body pose", result.robotRelativePose.toString());
            }
            telemetry.update();
        }

        robot.stopVision();
    }
}
