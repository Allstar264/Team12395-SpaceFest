package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.math.Pose;

public class UtilFunctions {
    public static double normalizeRadians(double radians) {
        radians %= 2.0 * Math.PI;

        if (radians >= Math.PI) radians -= 2.0 * Math.PI;
        if (radians < -Math.PI) radians += 2.0 * Math.PI;

        return radians;
    }

    public static Pose ftcPoseToPedro(Pose pose) {
        return new Pose(pose.y() + 72, 72 - pose.x(), pose.heading() - (Math.PI / 2));
    }

    public static Pose ftcPoseToPedro(Pose2d pose){
        return ftcPoseToPedro(new Pose(pose.position.x, pose.position.y, pose.heading.toDouble()));
    }

    public static Pose2d PedroPoseToFTCPose(Pose pose) {
        return new Pose2d(-pose.y() + 72, pose.x() - 72, pose.heading() + (Math.PI / 2));
    }
}
