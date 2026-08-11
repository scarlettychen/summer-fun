package org.firstinspires.ftc.teamcode.brainstem.auto;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.brainstem.follower.PathFollower;

public abstract class CommandAutoOpMode extends LinearOpMode {

    protected BrainSTEMRobot robot;
    protected PathFollower drive;

    protected abstract boolean isRed();

    protected double[] startPose() {
        return null;
    }

    protected abstract Command buildAuto();

    protected void addInitTelemetry() {}

    protected void addRunTelemetry() {}

    @Override
    public final void runOpMode() {
        robot = new BrainSTEMRobot(hardwareMap, telemetry, this);
        robot.setAlliance(isRed());
        double[] start = startPose();
        if (start != null) {
            robot.setStartPose(start);
        }
        drive = robot.createPathFollower();

        Command auto = buildAuto();

        for (int i = 0; i < 10 && opModeInInit(); i++) {
            robot.update();
            addInitTelemetry();
            telemetry.update();
            sleep(50);
        }

        waitForStart();
        if (isStopRequested()) return;

        if (start != null) {
            robot.setStartPose(start);
        }
        robot.update();

        Scheduler.reset();
        Scheduler.schedule(auto);

        while (opModeIsActive() && Scheduler.isScheduled(auto)) {
            robot.update();
            Scheduler.execute();
            addRunTelemetry();
            telemetry.update();
        }

        Scheduler.reset();
        drive.cancel();
        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.stopVision();
    }
}
