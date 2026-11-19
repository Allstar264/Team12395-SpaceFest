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
        c.xPodOffset.set(-1.73228346);
        c.yPodOffset.set(-4.16815);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });





    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.28045389467961046);
                Controller secondaryTranslationalForward = Controller.proportional(0.10362022021803372);
                Controller primaryTranslationalLateral = Controller.proportional(0.4845005173422135);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17900999506569293);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013149662350664093));
                c.brake.set(Controller.proportionalFeedforward(0.011177212998064479));

                c.headingFeedback.set(Controller.proportional(3.3409308341251513));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05321749862394655, 0.006915628866352109));

                c.linearBrakeCoefficients.set(Matrix.diag(0.054323852544685185, 0.0277383464580249));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0025882617803579862, 0.0030413990897900675));

                c.maxAchievableForwardVelocity.set(79.95753022423852);
                c.maxAchievableStrafeVelocity.set(65.60517193752062);
                c.naturalForwardDeceleration.set(36.24739850072219);
                c.naturalStrafeDeceleration.set(65.19620090542459);
            }
    );

}