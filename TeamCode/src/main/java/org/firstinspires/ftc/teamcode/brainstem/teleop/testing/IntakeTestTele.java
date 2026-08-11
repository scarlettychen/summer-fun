package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.auto.OpmodeCommands;
import org.firstinspires.ftc.teamcode.brainstem.subsystems.Intake;

@TeleOp(name = "intake commands", group = "Test")
public class IntakeTestTele extends SubsystemTestTele {

    private Intake intake;

    @Override
    protected void onInit() {
        intake = new Intake(hardwareMap, telemetry);
    }

    @Override
    protected void handleInput() {
        if (gp1.isFirstA()) {
            run(OpmodeCommands.setIntake(intake, Intake.IntakeState.IN));
        }
        if (gp1.isFirstB()) {
            run(OpmodeCommands.turnOffIntake(intake));
        }
        if (gp1.isFirstX()) {
            run(OpmodeCommands.reverseIntake(intake));
        }
    }

    @Override
    protected void onUpdate() {
        intake.update();
    }

    @Override
    protected void addTelemetry() {
        telemetry.addData("state", intake.getIntakeState());
    }
}
