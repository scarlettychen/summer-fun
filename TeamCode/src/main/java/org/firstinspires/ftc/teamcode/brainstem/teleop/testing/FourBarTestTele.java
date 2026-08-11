package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.auto.OpmodeCommands;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.FourBarLinkage;

@TeleOp(name = "Test Lift", group = "Test")
public class FourBarTestTele extends SubsystemTestTele {

    private FourBarLinkage lift;

    @Override
    protected void onInit() {
        lift = new FourBarLinkage(hardwareMap, telemetry);
    }

    @Override
    protected void handleInput() {
        if (gp1.isFirstA()) {
            run(OpmodeCommands.setLiftDown(lift));
        }
        if (gp1.isFirstX()) {
            run(OpmodeCommands.setLiftLow(lift));
        }
        if (gp1.isFirstY()) {
            run(OpmodeCommands.setLiftHigh(lift));
        }
    }

    @Override
    protected void onUpdate() {
        lift.update();
    }

    @Override
    protected void addTelemetry() {
        telemetry.addData("state", lift.getState());
        telemetry.addData("pos", lift.getPosition());
        telemetry.addData("atTarget", lift.atTarget());
    }
}
