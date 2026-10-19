package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name="Op", group="Op1")
public class teleOp extends OpMode {
    public hardware robot = new hardware();

    private Follower follower;

    boolean toggle = false;

    boolean toggle2 = false;

    boolean toggle3 = false;
    double position = 0;
    double hoodPosition = 0.75;

    boolean shooterclock = false;

    ElapsedTime shooterclockcounter = new  ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);

        // For a standalone tracking test: treat the starting location as zero.
        follower.setPose(new Pose(0, 0, 0));
    }
    @Override
    public void loop(){
            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    follower.pose().heading()
            );

            follower.manual(powers);
            follower.update();

            Pose robotPose = follower.pose();

        if (gamepad1.squareWasPressed()){
            toggle = !toggle;
        }

        if (gamepad1.triangleWasPressed()){
            toggle2 = !toggle2;
        }

        if(gamepad1.circleWasPressed()){
            toggle3 = !toggle3;
        }


        if (toggle) {
            robot.intakeServo.setPosition(.95);
        } else {
            robot.intakeServo.setPosition(0);
        }


        if(gamepad1.dpadUpWasPressed()){
            hoodPosition += .025;
        } else if (gamepad1.dpadDownWasPressed()){
            hoodPosition -= .025;
        }

        if (toggle3){
            robot.setFlyWheel(1500);
        }   else {
            robot.setFlyWheel(750);
        }

        if (gamepad1.left_bumper){
            shooterclock = true;
        } else if (gamepad1.right_bumper) {
            robot.intakeMotor.setPower(1);
        } else if (gamepad1.dpad_left){
            robot.intakeMotor.setPower(-1);
        } else {
            shooterclock = false;
            robot.intakeMotor.setPower(0);
        }


        if (gamepad2.left_trigger >= 0.1){
            position += gamepad2.left_trigger * 5;
        } else if (gamepad2.right_trigger >= 0.1){
            position -= gamepad2.right_trigger * 5;
        }

        if (shooterclock){
            if (!(shooterclockcounter.seconds() > 0)){
                shooterclockcounter.startTime();
            }

            if (shooterclockcounter.seconds() > 0.1){
                robot.turretstopperServo.setPosition(.5);
            }

            if (shooterclockcounter.seconds() >= 1.1){
                robot.intakeMotor.setPower(.85);

                shooterclockcounter.reset();
                shooterclock = false;
            }
        } else {
            robot.turretstopperServo.setPosition(0.80);
        }



        robot.hoodServo.setPosition(hoodPosition);



        telemetry.addData("turret degrees: ", position);
        telemetry.addData("Velocity", robot.flyWheel.getVelocity());
        telemetry.addData("Hood Angle", hoodPosition);
        telemetry.addData("Robot X (in)", robotPose.x());
        telemetry.addData("Robot Y (in)", robotPose.y());
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(robotPose.heading()));
        telemetry.update();


    }
}
