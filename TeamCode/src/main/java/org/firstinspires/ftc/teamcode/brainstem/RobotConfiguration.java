package org.firstinspires.ftc.teamcode.brainstem;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotConfiguration {

    public FollowerConstants createFollowerConstants() {
        return new FollowerConstants()
                .mass(12.0)
                .centripetalScaling(0.0)
                .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.01, 0))
                .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0, 0, 0))
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008, 0, 0, 0.6, 0));
    }

    public MecanumConstants createMecanumConstants() {
        return new MecanumConstants()
                .leftFrontMotorName("FL")
                .leftRearMotorName("BL")
                .rightFrontMotorName("FR")
                .rightRearMotorName("BR")
                .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
    }

    public PinpointConstants createPinpointConstants() {
        return new PinpointConstants()
                .hardwareMapName("odo")
                .distanceUnit(DistanceUnit.MM)
                .forwardPodY(-43.123)
                .strafePodX(71.718)
                .customEncoderResolution(19.894)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    public RobotModel createRobotModel() {
        return new RobotModel()
                .mass(12.0)
                .wheelRadius(1.8898)
                .motorFreeSpeed(32.67)
                .gearRatio(1.0)
                .frictionCoefficient(0.7)
                .maxAcceleration(80.0)
                .maxDeceleration(100.0)
                .maxAngularVelocity(6.0)
                .maxAngularAcceleration(20.0)
                .feedforward(0.05, 0.18, 0.003);
    }
}
