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
        c.xPodOffset.set(-3.09055118);
        c.yPodOffset.set(4.72440945);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3659263108394279);
                Controller secondaryTranslationalForward = Controller.proportional(0.13519999412406394);
                Controller primaryTranslationalLateral = Controller.proportional(1.7416320063993609);
                Controller secondaryTranslationalLateral = Controller.proportional(0.6434864891002637);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.019830814847178253));
                c.brake.set(Controller.proportionalFeedforward(0.016856192620101514));

                c.headingFeedback.set(Controller.proportional(5.530054407025469));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04874098077743439, 0.0074807595107575335));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06921192184527798, 0.0732853792847051));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016554712971380415, 0.0012522887194578408));

                c.maxAchievableForwardVelocity.set(54.548432323473584);
                c.maxAchievableStrafeVelocity.set(40.8534383471399);
                c.naturalForwardDeceleration.set(43.92874165971504);
                c.naturalStrafeDeceleration.set(74.07810591266718);
                c.headingDriveRatio.set(.7);
                c.translationalDeviationTolerance.set(2.5);
                c.headingDeviationTolerance.set(Math.toRadians(11.25));
                c.parametricTConstraint.set(0.025);
                c.translationalConstraint.set(0.1);
                c.headingConstraint.set(0.007);

            }
    );

}