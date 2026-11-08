package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );

    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left_motor");
        c.frontRightName.set("front_right_motor");
        c.backLeftName.set("back_left_motor");
        c.backRightName.set("back_right_motor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.19319477982408417);
        c.yPodOffset.set(-4.920559980737882);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });





    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.295156105001793);
                Controller secondaryTranslationalForward = Controller.proportional(0.10905229408178514);
                Controller primaryTranslationalLateral = Controller.proportional(0.5589259616616712);
                Controller secondaryTranslationalLateral = Controller.proportional(0.20650820805723416);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012395625858035176));
                c.brake.set(Controller.proportionalFeedforward(0.0105362819793299));

                c.headingFeedback.set(Controller.proportional(3.5890515212049245));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04187126056320708, 0.007491882518648947));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08410409064903382, 0.041745453329099604));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002005539204053434, 0.0027134938165819427));

                c.maxAchievableForwardVelocity.set(79.99725509814782);
                c.maxAchievableStrafeVelocity.set(63.08261510998195);
                c.naturalForwardDeceleration.set(31.482007379482);
                c.naturalStrafeDeceleration.set(65.03374782941398);
            }
    );

}