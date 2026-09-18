package org.firstinspires.ftc.teamcode;

public class UtilFunctions {
    public static double normalizeRadians(double radians) {
        radians %= 2.0 * Math.PI;

        if (radians >= Math.PI) radians -= 2.0 * Math.PI;
        if (radians < -Math.PI) radians += 2.0 * Math.PI;

        return radians;
    }
}
