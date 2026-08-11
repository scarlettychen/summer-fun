package org.firstinspires.ftc.teamcode.brainstem.logging;

public final class LogEntry {

    public static final int MAX_EXTRA = 32;

    public static final String CSV_HEADER =
            "timestamp_ms,loop_time_ms,"
                    + "x_in,y_in,heading_deg,velocity_in_s,battery_v,"

                    + "fl_power,fr_power,bl_power,br_power,"
                    + "fl_ticks,fr_ticks,bl_ticks,br_ticks,"
                    + "fl_tps,fr_tps,bl_tps,br_tps,"

                    + "turret_pos,turret_target,turret_error,"
                    + "lift_pos,lift_target,"

                    + "vision_has_target,vision_tx,vision_ty,vision_conf,vision_latency_ms,"

                    + "odo_x,odo_y,odo_h,"
                    + "vision_pose_x,vision_pose_y,vision_pose_h,"
                    + "fused_x,fused_y,fused_h";

    public long timestampMs;
    public double loopTimeMs;
    public double xIn;
    public double yIn;
    public double headingDeg;
    public double velocityInPerSec;
    public double batteryV;

    public double flPower, frPower, blPower, brPower;
    public double flTicks, frTicks, blTicks, brTicks;
    public double flTicksPerSec, frTicksPerSec, blTicksPerSec, brTicksPerSec;

    public double turretPos = Double.NaN;
    public double turretTarget = Double.NaN;
    public double turretError = Double.NaN;
    public double liftPos;
    public double liftTarget;

    public double visionHasTarget;
    public double visionTx = Double.NaN;
    public double visionTy = Double.NaN;
    public double visionConf = Double.NaN;
    public double visionLatencyMs = Double.NaN;

    public double odoX, odoY, odoH;
    public double visionPoseX = Double.NaN, visionPoseY = Double.NaN, visionPoseH = Double.NaN;
    public double fusedX, fusedY, fusedH;

    public final double[] extra = new double[MAX_EXTRA];
    public int extraCount;

    public void reset() {
        timestampMs = 0L;
        loopTimeMs = 0;
        xIn = yIn = headingDeg = velocityInPerSec = batteryV = 0;
        flPower = frPower = blPower = brPower = 0;
        flTicks = frTicks = blTicks = brTicks = 0;
        flTicksPerSec = frTicksPerSec = blTicksPerSec = brTicksPerSec = 0;
        turretPos = turretTarget = turretError = Double.NaN;
        liftPos = liftTarget = 0;
        visionHasTarget = 0;
        visionTx = visionTy = visionConf = visionLatencyMs = Double.NaN;
        odoX = odoY = odoH = 0;
        visionPoseX = visionPoseY = visionPoseH = Double.NaN;
        fusedX = fusedY = fusedH = 0;
        for (int i = 0; i < extraCount; i++) {
            extra[i] = Double.NaN;
        }
    }

    public void setExtra(int index, double value) {
        if (index >= 0 && index < extraCount) {
            extra[index] = value;
        }
    }

    public void appendCsv(StringBuilder sb) {
        sb.append(timestampMs).append(',')
                .append(loopTimeMs).append(',')
                .append(xIn).append(',')
                .append(yIn).append(',')
                .append(headingDeg).append(',')
                .append(velocityInPerSec).append(',')
                .append(batteryV).append(',')
                .append(flPower).append(',')
                .append(frPower).append(',')
                .append(blPower).append(',')
                .append(brPower).append(',')
                .append(flTicks).append(',')
                .append(frTicks).append(',')
                .append(blTicks).append(',')
                .append(brTicks).append(',')
                .append(flTicksPerSec).append(',')
                .append(frTicksPerSec).append(',')
                .append(blTicksPerSec).append(',')
                .append(brTicksPerSec).append(',')
                .append(turretPos).append(',')
                .append(turretTarget).append(',')
                .append(turretError).append(',')
                .append(liftPos).append(',')
                .append(liftTarget).append(',')
                .append(visionHasTarget).append(',')
                .append(visionTx).append(',')
                .append(visionTy).append(',')
                .append(visionConf).append(',')
                .append(visionLatencyMs).append(',')
                .append(odoX).append(',')
                .append(odoY).append(',')
                .append(odoH).append(',')
                .append(visionPoseX).append(',')
                .append(visionPoseY).append(',')
                .append(visionPoseH).append(',')
                .append(fusedX).append(',')
                .append(fusedY).append(',')
                .append(fusedH);
        for (int i = 0; i < extraCount; i++) {
            sb.append(',').append(extra[i]);
        }
    }
}
