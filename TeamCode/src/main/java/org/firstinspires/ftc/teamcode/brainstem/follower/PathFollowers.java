package org.firstinspires.ftc.teamcode.brainstem.follower;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.brainstem.RobotModel;

public final class PathFollowers {
    private PathFollowers() {}

    public static PathFollower pedro(Follower pedroFollower, RobotModel robotModel) {
        return new PedroFollowerAdapter(pedroFollower, robotModel);
    }
}
