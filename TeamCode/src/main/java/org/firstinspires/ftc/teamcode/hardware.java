package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedro.Constants;




public class hardware {
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, intakeMotor;
    public DcMotorEx flyWheel, turret;
    public IMU imu;
    public Servo intakeServo, turretstopperServo,hoodServo;

    public GoBildaPinpointDriver pinpoint;


    public hardware() {
    }

    public void init(HardwareMap hardware){
        frontLeftMotor = hardware.get(DcMotor.class, "front_left_motor"); // CH 0
        backLeftMotor = hardware.get(DcMotor.class, "back_left_motor"); // CH 1
        frontRightMotor = hardware.get(DcMotor.class, "front_right_motor"); // CH 2
        backRightMotor = hardware.get(DcMotor.class, "back_right_motor"); // CH 3
        intakeMotor = hardware.get(DcMotor.class, "intake_motor"); // EH 1
        flyWheel = hardware.get(DcMotorEx.class, "front_fly_wheel"); // EH 3
        turret = hardware.get(DcMotorEx.class, "turret"); // EH 2
        intakeServo = hardware.get(Servo.class, "intake_servo"); // EH 0
        turretstopperServo = hardware.get(Servo.class, "turret_servo"); // EH 2B
        hoodServo = hardware.get(Servo.class, "hood_servo"); // EH 1
        pinpoint = hardware.get(GoBildaPinpointDriver.class, "pinpoint"); // EH 0

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,        // e.g., logo pointing up
                RevHubOrientationOnRobot.UsbFacingDirection.UP));  // e.g., USB ports towards right

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


        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition(0);



        flyWheel.setVelocityPIDFCoefficients(8, 0.4, 0, 14);


    }


    public void driveFieldCentric(double axial, double lateral, double yaw) {
        // NOTE: Heading is in radians; positive CCW. We rotate the input vector by -heading.
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Input rotation for field frame
        double lateralRotation = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double axialRotation = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

        // WHY: Standard mecanum mixing (A + L + Y. etc.). Values may exceed |1|; we normalize below.
        double frontLeftPower  = axialRotation + lateralRotation + yaw;
        double frontRightPower = axialRotation - lateralRotation - yaw;
        double backLeftPower   = axialRotation - lateralRotation + yaw;
        double backRightPower  = axialRotation + lateralRotation - yaw;

        // Normalize so that the highest magnitude is 1.0, preserving ratios
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        setDrivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void setDrivePower(double frontLeftWheel, double frontRightWheel, double backLeftWheel, double backRightWheel) {
        frontLeftMotor.setPower(frontLeftWheel);
        frontRightMotor.setPower(frontRightWheel);
        backLeftMotor.setPower(backLeftWheel);
        backRightMotor.setPower(backRightWheel);

    }

    public void setFlyWheel(int velocity){
        flyWheel.setVelocity(velocity);
    }

    public void setTurret(double degrees) {
        degrees = Range.clip(degrees, -140, 140);
        double stepsPerRotation = 145.090909091 * (158./26);
        double targetPosition = (stepsPerRotation/360) * degrees;

        turret.setTargetPosition( (int) Math.round(targetPosition) );
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }

    public double turretaim (double goalx, double goaly, double robotx, double roboty, double heading ) {


        double dx = goalx - robotx;
        double dy = goaly - roboty;

        double angle = Math.atan2(dy, dx) - heading;
        angle = Math.atan2(Math.sin(angle), Math.cos(angle));

        return Math.toDegrees(angle);
    }



    public class PedroActions {
        public Command setshooterspeed(double speed) {
            return Command.build()
                    .setStart(() -> setFlyWheel((int) speed))
                    .setDone(() -> true);

        }
    }



    }



