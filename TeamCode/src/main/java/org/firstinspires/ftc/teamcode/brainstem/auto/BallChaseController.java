package org.firstinspires.ftc.teamcode.brainstem.auto;

import org.firstinspires.ftc.teamcode.brainstem.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.brainstem.utils.PIDController;

final class BallChaseController {
    private final Limelight limelight;
    private final PIDController turnPid = new PIDController(0, 0, 0);
    private final PIDController rangePid = new PIDController(0, 0, 0);

    BallChaseController(Limelight limelight) {
        this.limelight = limelight;
    }

    void reset() {
        turnPid.reset();
        rangePid.reset();
    }

    Limelight.BallDetection track() {
        limelight.update();
        turnPid.setGains(Limelight.CHASE_KP_TX, Limelight.CHASE_KI_TX, Limelight.CHASE_KD_TX);
        rangePid.setGains(Limelight.CHASE_KP_RANGE, Limelight.CHASE_KI_RANGE, Limelight.CHASE_KD_RANGE);
        turnPid.setOutputLimit(Limelight.CHASE_MAX_TURN);
        rangePid.setOutputLimit(Limelight.CHASE_MAX_FORWARD);
        return limelight.getClosestBall();
    }

    double rangeInches(Limelight.BallDetection ball) {
        return limelight.estimateRangeInches(ball.tyDeg);
    }

    double turnTo(double txDeg) {
        return -turnPid.calculate(txDeg, 0.0);
    }

    double forwardTo(double rangeInches) {
        if (Double.isNaN(rangeInches)) return 0.0;
        return Math.max(0.0, rangePid.calculate(Limelight.CHASE_STOP_RANGE_IN, rangeInches));
    }

    boolean isAimed(Limelight.BallDetection ball) {
        return Math.abs(ball.txDeg) <= Limelight.CHASE_TX_TOL_DEG;
    }

    boolean isClose(double rangeInches) {
        return !Double.isNaN(rangeInches) && rangeInches <= Limelight.CHASE_STOP_RANGE_IN;
    }
}
