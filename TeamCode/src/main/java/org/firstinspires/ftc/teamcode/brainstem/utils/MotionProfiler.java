package org.firstinspires.ftc.teamcode.brainstem.utils;

public class MotionProfiler {

    private double maxAccel, maxVel;

    private ProfileMode mode;
    private double targetPos = 0;
    private double startPos = 0;
    private double endPos = 0;
    private double startTime = 0;



    public enum ProfileMode {
        DIST,
        TIME,
        TURRET
    }

    public MotionProfiler(double maxAccel, double maxVel, ProfileMode mode) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
        this.mode = mode;
    }

    /** Refreshes the accel/vel limits, e.g. from live-tunable dashboard values. */
    public void setConstraints(double maxAccel, double maxVel) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
    }




    /** True once currentPosition is within tolerance of the target — for DIST/TURRET mode. */
    public boolean isFinished(double currentPosition, double tolerance) {
        return Math.abs(endPos - currentPosition) < tolerance;
    }

    /** True once totalTime (from getFullTimeState's clock) has fully elapsed — for TIME mode. */
    public boolean isFinished(double currentTime) {
        double rawDist = endPos - startPos;
        double dist = Math.abs(rawDist);
        double accelTime = maxVel / maxAccel;
        double accelDist = 0.5 * maxAccel * Math.pow(accelTime, 2);
        if (accelDist > (dist / 2)) {
            accelTime = Math.sqrt(dist / maxAccel);
        }
        double cruisingDist = dist - 2 * Math.min(accelDist, dist / 2);
        double cruisingTime = cruisingDist / maxVel;
        double totalTime = 2 * accelTime + cruisingTime;
        return (currentTime - startTime) >= totalTime;
    }

    public void setTarget(double newTarg, double currentPosition, double currentTime) {
        this.targetPos = newTarg;
        this.endPos = newTarg;
        this.startPos = currentPosition;
        this.startTime = currentTime;


    }

    public MotionState update(double currentPosition, double currentTime) {
        double distanceRemaining = targetPos - currentPosition;
        double elapsedTime = currentTime - startTime;

        if (mode == ProfileMode.TIME) {
            return calculateTimeBased(elapsedTime);
        } else {
            return calculateDistanceBased(distanceRemaining);
        }
    }

    public double update(double headingError) {
        return calculateTurretMotion(headingError);
    }

    private double calculateTurretMotion(double headingError) {
        double direction = Math.signum(headingError);
        double absDist = Math.abs(headingError);
        double vi = Math.sqrt(2 * maxAccel * absDist);

        if (vi > maxVel) vi = maxVel;


        return vi * direction;
    }

    // vf^2 = vi^2 - 2ad
    //vi = sqrt(2 * a * d)
    // kV * this velco

    private MotionState calculateDistanceBased(double distanceRemaining) {
        double rawDist = endPos - startPos;
        double dist = Math.abs(rawDist);
        // Direction of travel is fixed by where the profile starts/ends, not by
        // live position feedback — using signum(distanceRemaining) here used to
        // flip the whole profile's sign to 0 exactly at the target and to
        // negative just past it, turning any small overshoot into an
        // ever-growing reverse velocity/acceleration command that never settles.
        double sign = Math.signum(rawDist);
        double peakVel = maxVel;

        double pos, vel, accel;

        double accelTime = maxVel / maxAccel;
        double accelDist = 0.5 * maxAccel * Math.pow(accelTime, 2);

        if (accelDist > (dist / 2)) {
            accelDist = dist / 2;
            peakVel = Math.sqrt(2 * maxAccel * accelDist);
        }

        double absDistRemaining = Math.abs(distanceRemaining);
        double distTraveled = Math.max(0, dist - absDistRemaining);

        if (absDistRemaining <= 0) {
            // arrived — hold at rest instead of reporting -maxAccel forever
            pos = endPos;
            vel = 0;
            accel = 0;
        } else if (distTraveled < accelDist) {
            // accelerating
            pos = startPos + (distTraveled * sign);
            vel = Math.sqrt(2 * maxAccel * distTraveled);
            accel = maxAccel;
        } else if (absDistRemaining > accelDist) {
            // cruising
            pos = startPos + (distTraveled * sign);
            vel = peakVel;
            accel = 0;
        } else {
            // decelerating
            pos = startPos + (distTraveled * sign);
            vel = Math.sqrt(2 * maxAccel * absDistRemaining);
            accel = -maxAccel;
        }

        if (Double.isNaN(vel)) vel = 0;

        return new MotionState(pos, vel * sign, accel * sign);
    }



    private MotionState calculateTimeBased(double time) {
        double rawDist = endPos - startPos;
        double dist = Math.abs(rawDist);
        double sign = Math.signum(rawDist);
        double peakVel = maxVel;

        double pos = 0, vel = 0, accel = 0;


        double accelTime = maxVel/maxAccel;
        double accelDist = 0.5 * maxAccel * Math.pow(accelTime, 2);

        if (accelDist > (dist / 2)) {
            accelTime = Math.sqrt(dist / maxAccel);
            accelDist = 0.5 * maxAccel * Math.pow(accelTime, 2); // recalculate distance
            peakVel = maxAccel * accelTime; }

        double cruisingDist = dist - 2 * accelDist;
        double cruisingTime = cruisingDist / peakVel;

        double decelStartTime = accelTime + cruisingTime;
        double totalTime = decelStartTime + accelTime;

        if (time > totalTime) {
            pos = dist;
            vel = 0;
            accel = 0;
        }

        // accelling
        if (time < accelTime) {
            pos = 0.5 * maxAccel * Math.pow(time, 2);
            vel = maxAccel * time;
            accel = maxAccel;
        }
        //cruising
        else if (time >= accelTime && time < decelStartTime) {
            double cruiseCurrentTime = time - accelTime; // Time spent cruising so far
            pos = accelDist + (peakVel * cruiseCurrentTime);
            vel = peakVel;
            accel = 0;
        }
        // decelling
        else if (time >= decelStartTime && time <= totalTime) {
            double decelCurrentTime = time - decelStartTime; // Time spent decelerating so far
            pos = accelDist + cruisingDist + (peakVel * decelCurrentTime) - (0.5 * maxAccel * Math.pow(decelCurrentTime, 2));
            vel = peakVel - (maxAccel * decelCurrentTime);
            accel = -maxAccel;
        }

        if (Double.isNaN(vel)) {
            vel = 0;
        }

        return new MotionState(
                startPos + (pos * sign),
                vel * sign,
                accel * sign
        );
    }

    public MotionState getFullTimeState(double currentTime) {
        return calculateTimeBased(currentTime - startTime);
    }

    public double getTargetPos() {
        return targetPos;
    }
    public void updateTarget(double newTarg) {
        this.targetPos = newTarg;
        this.endPos = newTarg;
    }

    public static class MotionState {
        public double position;
        public double velocity;
        public double acceleration;

        public MotionState(double position, double velocity, double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }
}
