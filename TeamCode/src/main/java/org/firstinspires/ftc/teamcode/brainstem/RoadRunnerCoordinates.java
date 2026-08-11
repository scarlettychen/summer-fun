package org.firstinspires.ftc.teamcode.brainstem;

import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.FTCCoordinates;

public enum RoadRunnerCoordinates implements CoordinateSystem {
    INSTANCE;

    private static final double TEAM_ZERO_OFFSET = Math.PI / 2;

    @Override
    public Pose convertToPedro(Pose pose) {

        Pose ftc = new Pose(pose.getX(), pose.getY(), pose.getHeading() + TEAM_ZERO_OFFSET);
        return FTCCoordinates.INSTANCE.convertToPedro(ftc);
    }

    @Override
    public Pose convertFromPedro(Pose pose) {
        Pose ftc = FTCCoordinates.INSTANCE.convertFromPedro(pose);
        return new Pose(ftc.getX(), ftc.getY(), ftc.getHeading() - TEAM_ZERO_OFFSET, INSTANCE);
    }
}
