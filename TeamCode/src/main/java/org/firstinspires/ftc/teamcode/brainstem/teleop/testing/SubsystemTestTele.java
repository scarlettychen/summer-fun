package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.brainstem.utils.GamepadTracker;

public abstract class SubsystemTestTele extends LinearOpMode {

    protected GamepadTracker gp1;
    protected GamepadTracker gp2;

    protected abstract void onInit();

    protected abstract void handleInput();

    protected abstract void onUpdate();

    protected abstract void addTelemetry();

    protected final void run(Command command) {
        Scheduler.schedule(command);
    }

    @Override
    public final void runOpMode() {
        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);
        onInit();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            gp1.update();
            gp2.update();
            handleInput();
            Scheduler.execute();
            onUpdate();
            addTelemetry();
            telemetry.update();
        }

        Scheduler.reset();
    }
}
