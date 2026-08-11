package org.firstinspires.ftc.teamcode.brainstem.subsystems;

import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.brainstem.RobotConfiguration;

public class Drive {

    public static final double STICK_XY_SCALE = 0.99;
    public static final double STICK_RX_SCALE = 0.75;

    private final DcMotorEx FL;
    private final DcMotorEx FR;
    private final DcMotorEx BL;
    private final DcMotorEx BR;

    public Drive(HardwareMap hardwareMap, RobotConfiguration configuration) {
        MecanumConstants cfg = configuration.createMecanumConstants();

        FL = hardwareMap.get(DcMotorEx.class, cfg.leftFrontMotorName);
        FR = hardwareMap.get(DcMotorEx.class, cfg.rightFrontMotorName);
        BL = hardwareMap.get(DcMotorEx.class, cfg.leftRearMotorName);
        BR = hardwareMap.get(DcMotorEx.class, cfg.rightRearMotorName);

        FL.setDirection(cfg.leftFrontMotorDirection);
        FR.setDirection(cfg.rightFrontMotorDirection);
        BL.setDirection(cfg.leftRearMotorDirection);
        BR.setDirection(cfg.rightRearMotorDirection);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setMotorPowers(0, 0, 0, 0);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        FR.setPower(fr);
        FL.setPower(fl);
        BR.setPower(br);
        BL.setPower(bl);
    }

    public void driveArcade(double y, double x, double rx) {
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }
        setMotorPowers(fl, fr, bl, br);
    }

    public void driveFromSticks(double leftStickY, double leftStickX, double rightStickX) {
        driveArcade(-leftStickY * STICK_XY_SCALE, leftStickX * STICK_XY_SCALE, rightStickX * STICK_RX_SCALE);
    }

    public void copyDriveLog(org.firstinspires.ftc.teamcode.brainstem.logging.LogEntry e) {
        e.flPower = FL.getPower();
        e.frPower = FR.getPower();
        e.blPower = BL.getPower();
        e.brPower = BR.getPower();
        e.flTicks = FL.getCurrentPosition();
        e.frTicks = FR.getCurrentPosition();
        e.blTicks = BL.getCurrentPosition();
        e.brTicks = BR.getCurrentPosition();
        e.flTicksPerSec = FL.getVelocity();
        e.frTicksPerSec = FR.getVelocity();
        e.blTicksPerSec = BL.getVelocity();
        e.brTicksPerSec = BR.getVelocity();
    }
}
