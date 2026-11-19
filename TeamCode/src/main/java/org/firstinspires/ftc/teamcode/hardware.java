package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import java.util.ArrayList;
import java.util.Arrays;

public class hardware {

    public DcMotor frontLeftMotor, backLeftMotor;
    public DcMotor frontRightMotor, backRightMotor, intakeMotor;

    public DcMotorEx flyWheel, turret;
    public IMU imu;
    public Servo intakeServo, turretstopperServo;
    public GoBildaPinpointDriver pinpoint;

    public RevColorSensorV3 colorsensor1;
    public RevColorSensorV3 colorsensor2;

    public OpMode myOpMode;

    // Turret pivot offset FROM the robot's localization origin.
    // Assumes localization tracks the robot center.
    // Units: inches.
    // Positive forward = toward robot front.
    // Positive left = toward robot left.
    //
    // Current setting assumes the turret is 14.727 mm BEHIND center.
    public double turretForwardOffset = -14.727 / 25.4;
    public double turretLeftOffset = 0.0;

    public hardware() {
    }

    public void init(HardwareMap hardware) {
        frontLeftMotor = hardware.get(
                DcMotor.class, "front_left_motor");

        backLeftMotor = hardware.get(
                DcMotor.class, "back_left_motor");

        frontRightMotor = hardware.get(
                DcMotor.class, "front_right_motor");

        backRightMotor = hardware.get(
                DcMotor.class, "back_right_motor");

        intakeMotor = hardware.get(
                DcMotor.class, "intake_motor");

        flyWheel = hardware.get(
                DcMotorEx.class, "front_fly_wheel");

        turret = hardware.get(
                DcMotorEx.class, "turret");

        intakeServo = hardware.get(
                Servo.class, "intake_servo");

        turretstopperServo = hardware.get(
                Servo.class, "stopper_servo");

        pinpoint = hardware.get(
                GoBildaPinpointDriver.class, "pinpoint");

        colorsensor1 = hardware.get(
                RevColorSensorV3.class, "color_sensor1");

        colorsensor2 = hardware.get(
                RevColorSensorV3.class, "color_sensor2");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu = hardware.get(IMU.class, "imu");
        imu.initialize(parameters);

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Point the shooter straight forward before INIT.
        // This sets its current position to encoder zero.
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition(0);

        flyWheel.setVelocityPIDFCoefficients(50, 0, 0, 14.5);
    }

    public void driveFieldCentric(double axial, double lateral, double yaw) {
        double botHeading = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);

        double lateralRotation =
                lateral * Math.cos(-botHeading)
                        - axial * Math.sin(-botHeading);

        double axialRotation =
                lateral * Math.sin(-botHeading)
                        + axial * Math.cos(-botHeading);

        double frontLeftPower = axialRotation + lateralRotation + yaw;
        double frontRightPower = axialRotation - lateralRotation - yaw;
        double backLeftPower = axialRotation - lateralRotation + yaw;
        double backRightPower = axialRotation + lateralRotation - yaw;

        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        setDrivePower(
                frontLeftPower,
                frontRightPower,
                backLeftPower,
                backRightPower
        );
    }

    public void setDrivePower(
            double frontLeftWheel,
            double frontRightWheel,
            double backLeftWheel,
            double backRightWheel) {

        frontLeftMotor.setPower(frontLeftWheel);
        frontRightMotor.setPower(frontRightWheel);
        backLeftMotor.setPower(backLeftWheel);
        backRightMotor.setPower(backRightWheel);
    }

    public void setFlyWheel(int velocity) {
        flyWheel.setVelocity(velocity);
    }

    public void setTurretstopperServo(double angle) {
        turretstopperServo.setPosition(angle);
    }

    public void setTurret(double degrees) {
        degrees = Range.clip(degrees, -160, 160);

        double stepsPerRotation = 145.090909091 * (158.0 / 26.0);
        double targetPosition = (stepsPerRotation / 360.0) * degrees;

        turret.setTargetPosition((int) Math.round(targetPosition));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.65);
    }

    public double turretaim(
            double goalx,
            double goaly,
            double robotx,
            double roboty,
            double heading) {

        // Heading is in radians, positive counterclockwise.
        // Rotate the turret's robot-relative offset into field coordinates.
        double turretX = robotx
                + turretForwardOffset * Math.cos(heading)
                - turretLeftOffset * Math.sin(heading);

        double turretY = roboty
                + turretForwardOffset * Math.sin(heading)
                + turretLeftOffset * Math.cos(heading);

        // Calculate direction from the actual turret pivot.
        double dx = goalx - turretX;
        double dy = goaly - turretY;

        // Convert field direction into a robot-relative angle.
        double angle = Math.atan2(dy, dx) - heading;

        // Wrap to -PI through PI.
        angle = Math.atan2(Math.sin(angle), Math.cos(angle));

        return Math.toDegrees(angle);
    }

     private ArrayList<double[]> velDataPoints = new ArrayList<>(Arrays.asList(
            new double[]{50,1450},
            new double[]{50,1350},
            new double[]{50,1450}


    ));

    private final double[] slopeList = initializeSlopeList();

    private double[] initializeSlopeList() {
        int maxIndex = velDataPoints.size();
        double[] list = new double[maxIndex];

        for (int i = 0; i < maxIndex - 1; i++) {
            list[i] = (
                    (velDataPoints.get(i + 1)[1] - velDataPoints.get(i)[1]) /
                            (velDataPoints.get(i + 1)[0] - velDataPoints.get(i)[0])
            );
        }

        return list;
    }

    public double getCalculatedVelocity(double distance) {
        double returnVal = 500;

        double velZeroIntercept =
                velDataPoints.get(0)[1] - slopeList[0] * velDataPoints.get(0)[0];

        for (int i = 0; i < slopeList.length; i++) {
            if (i == 0) {
                if (distance >= 0 && distance <= velDataPoints.get(0)[0]) {
                    myOpMode.telemetry.addData(
                            "Top Reference: ",
                            velDataPoints.get(i)[0] + ", " + velDataPoints.get(i)[1]
                    );

                    myOpMode.telemetry.addData(
                            "Slope Reference: ", slopeList[i]
                    );

                    returnVal = velZeroIntercept + distance * slopeList[0];
                }
            } else if (
                    (i == slopeList.length - 1
                            && distance >= velDataPoints.get(i)[0])
                    || distance > velDataPoints.get(i)[0]
                    && distance <= velDataPoints.get(i + 1)[0]) {

                myOpMode.telemetry.addData(
                        "Bottom Reference: ",
                        velDataPoints.get(i)[0] + ", " + velDataPoints.get(i)[1]
                );

                myOpMode.telemetry.addData(
                        "Slope Reference: ", slopeList[i]
                );

                returnVal = velDataPoints.get(i)[1]
                        + (distance - velDataPoints.get(i)[0]) * slopeList[i];
            }
        }

        return returnVal;
    }
}