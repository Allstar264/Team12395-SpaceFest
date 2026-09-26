package org.firstinspires.ftc.teamcode;

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
}
