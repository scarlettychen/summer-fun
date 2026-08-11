package org.firstinspires.ftc.teamcode.brainstem.utils;

import java.util.function.DoubleSupplier;

public class AntiTipping {

    public static final class DriveCorrection {
        public final double forward;
        public final double strafe;

        public DriveCorrection(double forward, double strafe) {
            this.forward = forward;
            this.strafe = strafe;
        }

        public static final DriveCorrection NONE = new DriveCorrection(0, 0);
    }

    private final DoubleSupplier pitchDegrees;
    private final DoubleSupplier rollDegrees;

    private double kP;
    private double enterThresholdDegrees;
    private double exitThresholdDegrees;
    private double maxCorrectionPower;
    private double minCorrectionPower;
    private double axisDeadbandDegrees = 2.0;

    private double pitchOffsetDegrees;
    private double rollOffsetDegrees;

    private boolean tipping = false;
    private DriveCorrection lastCorrection = DriveCorrection.NONE;

    public AntiTipping(DoubleSupplier pitchDegrees, DoubleSupplier rollDegrees,
                       double kP, double tippingThresholdDegrees, double maxCorrectionPower) {
        this(pitchDegrees, rollDegrees, kP, tippingThresholdDegrees,
                tippingThresholdDegrees * 0.35, maxCorrectionPower, 0.35);
    }

    public AntiTipping(DoubleSupplier pitchDegrees, DoubleSupplier rollDegrees,
                       double kP, double enterThresholdDegrees, double exitThresholdDegrees,
                       double maxCorrectionPower, double minCorrectionPower) {
        this.pitchDegrees = pitchDegrees;
        this.rollDegrees = rollDegrees;
        this.kP = kP;
        this.enterThresholdDegrees = enterThresholdDegrees;
        this.exitThresholdDegrees = Math.min(exitThresholdDegrees, enterThresholdDegrees);
        this.maxCorrectionPower = maxCorrectionPower;
        this.minCorrectionPower = minCorrectionPower;
    }

    public void tare() {
        pitchOffsetDegrees = pitchDegrees.getAsDouble();
        rollOffsetDegrees = rollDegrees.getAsDouble();
        tipping = false;
        lastCorrection = DriveCorrection.NONE;
    }

    public void tare(double pitchDegrees, double rollDegrees) {
        this.pitchOffsetDegrees = pitchDegrees;
        this.rollOffsetDegrees = rollDegrees;
        tipping = false;
        lastCorrection = DriveCorrection.NONE;
    }

    public DriveCorrection calculate() {
        double pitchErr = pitchDegrees.getAsDouble() - pitchOffsetDegrees;
        double rollErr = rollDegrees.getAsDouble() - rollOffsetDegrees;
        double magnitude = Math.max(Math.abs(pitchErr), Math.abs(rollErr));

        if (tipping) {
            tipping = magnitude > exitThresholdDegrees;
        } else {
            tipping = magnitude > enterThresholdDegrees;
        }

        if (!tipping) {
            lastCorrection = DriveCorrection.NONE;
            return lastCorrection;
        }

        double forward = axisCorrection(pitchErr);
        double strafe = axisCorrection(rollErr);

        lastCorrection = new DriveCorrection(forward, strafe);
        return lastCorrection;
    }

    private double axisCorrection(double angleErrorDeg) {
        if (Math.abs(angleErrorDeg) < axisDeadbandDegrees) {
            return 0;
        }
        double raw = -kP * angleErrorDeg;

        if (Math.abs(raw) < minCorrectionPower) {
            raw = Math.copySign(minCorrectionPower, raw);
        }
        return clamp(raw, -maxCorrectionPower, maxCorrectionPower);
    }

    public boolean isTipping() {
        return tipping;
    }

    public DriveCorrection getLastCorrection() {
        return lastCorrection;
    }

    public double getPitchErrorDegrees() {
        return pitchDegrees.getAsDouble() - pitchOffsetDegrees;
    }

    public double getRollErrorDegrees() {
        return rollDegrees.getAsDouble() - rollOffsetDegrees;
    }

    public void setTippingThreshold(double enterDegrees, double exitDegrees) {
        this.enterThresholdDegrees = enterDegrees;
        this.exitThresholdDegrees = Math.min(exitDegrees, enterDegrees);
    }

    public void setMaxCorrectionPower(double power) {
        this.maxCorrectionPower = power;
    }

    public void setMinCorrectionPower(double power) {
        this.minCorrectionPower = Math.max(0, power);
    }

    public void setKP(double kP) {
        this.kP = kP;
    }

    public void setAxisDeadbandDegrees(double degrees) {
        this.axisDeadbandDegrees = Math.max(0, degrees);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
