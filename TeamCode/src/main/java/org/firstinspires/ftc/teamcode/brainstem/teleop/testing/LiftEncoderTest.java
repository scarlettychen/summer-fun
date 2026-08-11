package org.firstinspires.ftc.teamcode.brainstem.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brainstem.subsystems.FourBarLinkage;

@TeleOp(name = "Lift encoder test", group = "Test")
public class LiftEncoderTest extends LinearOpMode {

    private FourBarLinkage lift;

    @Override
    public void runOpMode() {

        lift = new FourBarLinkage(hardwareMap, telemetry);
        lift.setState(FourBarLinkage.LinkState.OFF);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            lift.setState(FourBarLinkage.LinkState.TESTING);

            telemetry.addData("encoder", lift.getRightPosition());
            telemetry.update();

            lift.update();

        }
    }
}
