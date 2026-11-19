package org.firstinspires.ftc.teamcode.Auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.teleOp.velocity;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.PedroActions;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoRed extends LinearOpMode {

    public hardware robot = new hardware();
    public PedroActions library = new  PedroActions();

    private Follower follower;
    private Command turretTracking;

    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose shootingPos1 = poseFactory.of(52, 5.4, 90);
    private final Pose flower1 = poseFactory.of(32, 47.5, 180);
    private final Pose flower1Control1 = poseFactory.of(45.3826, 45.7812, 0);
    private final Pose flower1In = poseFactory.of(12.9, 47.5, 180);
    private final Pose flower1Out = poseFactory.of(19, 47.5, 180);
    private final Pose shoot2Start = poseFactory.of(19, 47.5, 180);
    private final Pose shoot2 = poseFactory.of(60, 133.5, 270);
    private final Pose shoot2Control1 = poseFactory.of(23.12, 107.87, 0);
    private final Pose shoot2Control2 = poseFactory.of(48.8832, 98.4591, 0);
    private final Pose flower2Start = poseFactory.of(58, 135.9, 270);
    private final Pose flower2 = poseFactory.of(48.5, 120, 90);
    private final Pose flower2Control1 = poseFactory.of(62.0774, 115.8796, 0);
    private final Pose flower2In = poseFactory.of(48.5, 129, 90);
    private final Pose flower2out = poseFactory.of(48.5, 120, 90);
    private final Pose shoot3Start = poseFactory.of(48.5, 120, 90);
    private final Pose shoot3 = poseFactory.of(60, 133.5, 270);
    private final Pose shoot3Control1 = poseFactory.of(61.731, 115.7027, 0);
    private final Pose parkStart = poseFactory.of(58, 133, 270);
    private final Pose park = poseFactory.of(8, 95.1961, 0);
    private final Pose parkControl1 = poseFactory.of(46.6188, 92.3607, 0);
    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                library.setshooterspeed(1400),
                library.flowerservo(.15),
                library.servostopper(.6), //open
                waitMs(3000),
                library.intake(.8), //maybe slow down to .9
                waitMs(750),
                follow(follower, flower1()),
                library.flowerservo(.43), //down
                library.servostopper(.3), //close
                waitMs(250),
                follow(follower, flower1In()),
                waitMs(1500),
                follow(follower, flower1Out()),
                library.flowerservo(.15),
                follow(follower, shoot2()),
                library.intake(0),
                library.servostopper(.6),
                waitMs(1000),
                library.intake(1),
                waitMs(1000),
                library.servostopper(.3),
                library.flowerservo(.43),
                follow(follower, flower2()),
                library.intake(1),
                follow(follower, flower2In()),
                waitMs(1500),
                follow(follower, flower2out()),
                follow(follower, shoot3()),
                waitMs(1000),
                library.intake(0),
                library.servostopper(.6),
                library.flowerservo(.15),
                library.intake(1),
                waitMs(1000),
                follow(follower, park())

        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        robot.init(hardwareMap);
        library.initialize(robot);

        follower = Constants.create(hardwareMap);
        follower.setPose(shootingPos1);
        follower.update();

        // Track (58, 54) below Y=67 and (58, 94) above Y=77.
        // Keep the selected hive between those thresholds and update aim continuously.
        turretTracking = library.trackTurretHives(follower, 58, 54, 90);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        try {
            // Runs continuously alongside the autonomous routine.
            schedule(turretTracking);
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();

                OpModeStorage.autonomousEndPose = follower.pose();

                Scheduler.execute();

                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading", follower.pose().heading());
                telemetry.addData("velocity", velocity);
                telemetry.addData(
                        "Turret encoder",
                        robot.turret.getCurrentPosition()
                );
                telemetry.addData(
                        "Turret target",
                        robot.turret.getTargetPosition()
                );

                if (follower.currentPath() != null) {
                    telemetry.addData(
                            "Current path distance remaining",
                            follower.distanceToEndpoint()
                    );
                    telemetry.addData(
                            "Path number",
                            follower.pathIndex()
                    );
                }

                telemetry.update();
            }
        } finally {
            Scheduler.cancel(turretTracking);
            Scheduler.reset();

            robot.turret.setPower(0);
            robot.flyWheel.setPower(0);
            robot.intakeMotor.setPower(0);
            robot.setDrivePower(0, 0, 0, 0);
        }
    }

    public Path flower1() {
        return Paths.curve(shootingPos1, flower1Control1, flower1).linear(shootingPos1, flower1);
    }

    public Path flower1In() {
        return Paths.line(flower1, flower1In).constant(flower1In);
    }

    public Path flower1Out() {
        return Paths.line(flower1In, flower1Out).constant(flower1Out);
    }

    public Path shoot2() {
        return Paths.curve(shoot2Start, shoot2Control1, shoot2Control2, shoot2).linear(shoot2Start, shoot2);
    }

    public Path flower2() {
        return Paths.curve(flower2Start, flower2Control1, flower2).linear(flower2Start, flower2);
    }

    public Path flower2In() {
        return Paths.line(flower2, flower2In).constant(flower2In);
    }

    public Path flower2out() {
        return Paths.line(flower2In, flower2out).constant(flower2out);
    }

    public Path shoot3() {
        return Paths.curve(shoot3Start, shoot3Control1, shoot3).linear(shoot3Start, shoot3);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }

}