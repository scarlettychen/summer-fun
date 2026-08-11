package org.firstinspires.ftc.teamcode.brainstem.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP, kI, kD;

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTimestamp = 0;

    private double integralMax = Double.MAX_VALUE;
    private double outputMax = 1.0;

    private ElapsedTime timer = new ElapsedTime();

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        timer.reset();
        lastTimestamp = 0;
    }

    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setIntegralLimit(double max) {
        this.integralMax = max;
    }

    public void setOutputLimit(double max) {
        this.outputMax = max;
    }

    public double calculate(double current, double target) {
        double error = target - current;

        double currentTimestamp = timer.seconds();
        double dt = currentTimestamp - lastTimestamp;
        if (dt <= 0) dt = 1e-3;

        integralSum += error * dt;
        integralSum = Math.max(-integralMax, Math.min(integralMax, integralSum));

        double derivative = (error - lastError) / dt;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);
        output = Math.max(-outputMax, Math.min(outputMax, output));

        lastError = error;
        lastTimestamp = currentTimestamp;

        return output;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
        lastTimestamp = 0;
    }

    public double getLastError() {
        return lastError;
    }
}
