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

    boolean flowertoggle = false;


    boolean flyhweeltoggle = false;

    boolean turretoggle = false;
    double position = 0;

    public static int velocity = 0;

    public static double hoodPosition = 0;




    boolean shooterclock = false;

    public static Pose targetPoint = new Pose(58, 54);


    double goalx = 58;

    double goaly = 54;

    double angle = 0;
    double distanceToTarget;


    ElapsedTime shooterclockcounter = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);

        follower.setPose(new Pose(0, 0, 0));
    }

    @Override
    public void start(){
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
    }

    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading()
        );

        follower.manual(powers);


        if (gamepad1.xWasPressed()) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            follower.setPose(cornerPose);

        }
            follower.update();

        Pose robotPose = follower.pose();

        if (robotPose.y() < 67) {
            angle = robot.turretaim(goalx, goaly, robotPose.x(), robotPose.y(), robotPose.heading());
            targetPoint = new Pose(goalx, goaly, 0);
        } else if (robotPose.y() > 77) {
            angle = robot.turretaim(goalx, goaly + 40, robotPose.x(), robotPose.y(), robotPose.heading());
            targetPoint = new Pose(goalx, goaly + 40, 0);
        }

        distanceToTarget = Math.abs(targetPoint.distance(robotPose));

            if (gamepad1.squareWasPressed()) {
                flowertoggle = !flowertoggle;
            }

            if (gamepad1.circleWasPressed()) {
                flyhweeltoggle = !flyhweeltoggle;
            }

            if (gamepad1.triangleWasPressed()) {
                turretoggle = !turretoggle;
            }


            if (flowertoggle) {
                robot.intakeServo.setPosition(.95);
            } else {
                robot.intakeServo.setPosition(0);
            }



       /* if (flyhweeltoggle) {
            velocity  = robot.getCalculatedVelocity(distanceToTarget);
            telemetry.addData("Calculated Velocity: ", velocity);
        }




        robot.setFlyWheel((int) velocity);


        */

            if (gamepad1.dpadUpWasPressed()) {
                velocity += 50;
            } else if (gamepad1.dpadDownWasPressed()) {
                velocity -= 50;
                }






            if (gamepad1.left_bumper) {
                shooterclock = true;
            } else if (gamepad1.right_bumper) {
                robot.intakeMotor.setPower(1);
            } else if (gamepad1.dpad_left) {
                robot.intakeMotor.setPower(-1);
            } else {
                shooterclock = false;
                robot.intakeMotor.setPower(0);
            }


            if (gamepad2.left_trigger >= 0.1) {
                position += gamepad2.left_trigger * 5;
            } else if (gamepad2.right_trigger >= 0.1) {
                position -= gamepad2.right_trigger * 5;
            }

            if (shooterclock) {
                if (!(shooterclockcounter.seconds() > 0)) {
                    shooterclockcounter.startTime();
                }

                if (shooterclockcounter.seconds() > 0.1) {
                    robot.turretstopperServo.setPosition(.5);
                }

                if (shooterclockcounter.seconds() >= 1.1) {
                    robot.intakeMotor.setPower(.85);

                    shooterclockcounter.reset();
                    shooterclock = false;
                }
            } else {
                robot.turretstopperServo.setPosition(0.80);
            }



            if (turretoggle) {
                robot.setTurret(angle);
            } else {
                robot.setTurret(0);
            }


            robot.setFlyWheel(velocity);


            telemetry.addData("Velocity", robot.flyWheel.getVelocity());
            telemetry.addData("Hood Angle", hoodPosition);
            telemetry.addData("Robot X (in)", robotPose.x());
            telemetry.addData("Robot Y (in)", robotPose.y());
            telemetry.addData("Robot Heading (deg)", Math.toDegrees(robotPose.heading()));
            telemetry.addData("Turret Position", angle);
            telemetry.addData("Turret target", robot.turret.getTargetPosition());
            telemetry.addData(("Distance to Target"), distanceToTarget);
            telemetry.update();


        }
    }

