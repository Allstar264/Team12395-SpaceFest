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
        c.frontLeftName.set("front_left_drive");
        c.frontRightName.set("front_right_drive");
        c.backLeftName.set("back_left_drive");
        c.backRightName.set("back_right_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(4.72440945);
        c.yPodOffset.set(-3.09055118);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });





    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.404975179018142);
                Controller secondaryTranslationalForward = Controller.proportional(0.1496275075111244);
                Controller primaryTranslationalLateral = Controller.proportional(0.8209229904732503);
                Controller secondaryTranslationalLateral = Controller.proportional(0.30330910951356926);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.022433407668432385));
                c.brake.set(Controller.proportionalFeedforward(0.019068396518167528));

                c.headingFeedback.set(Controller.proportional(6.375127989281238));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04069804385642566, 0.009726342564428277));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06964685789621508, 0.041781112318786895));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001652595293591277, 0.002301529030056234));

                c.maxAchievableForwardVelocity.set(49.41690446091947);
                c.maxAchievableStrafeVelocity.set(33.44154580007893);
                c.naturalForwardDeceleration.set(41.75229977497882);
                c.naturalStrafeDeceleration.set(78.21150058833118);
                c.maxPathSpeed.set(1.0);
                c.headingDriveRatio.set(0.7);
                c.brakeAggression.set(1.05);
            }
    );

}