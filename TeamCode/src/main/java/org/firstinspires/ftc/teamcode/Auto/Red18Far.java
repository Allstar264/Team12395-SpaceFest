package org.firstinspires.ftc.teamcode.Auto;

import android.util.SparseArray;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware;
import  org.firstinspires.ftc.teamcode.pedro.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name="Red Pedro", group="Auto")
public class Red18Far extends LinearOpMode {
    private Follower follower;
    Hardware robot = new Hardware(this);
    Hardware.PedroActions library = robot.new PedroActions();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start1stShootingPos = poseFactory.of(56, 9.3, 90);
    private final Pose firstBall = poseFactory.of(10, 9.3, 180);
    private final Pose shootingPos2 = poseFactory.of(56, 9.3, 180);
    private final Pose point3rdSpike = poseFactory.of(44, 35, 180);
    private final Pose point3rdSpikeControl1 = poseFactory.of(59.5714, 33.6951, 0);
    private final Pose endof3rdSpike = poseFactory.of(9, 35, 180);
    private final Pose shootingPos3 = poseFactory.of(56, 9.3, 180);
    private final Pose shootingPos3Control1 = poseFactory.of(59.1886, 34.8251, 0);
    private final Pose point61stTunnelIntake = poseFactory.of(9, 28, 180);
    private final Pose point61stTunnelIntakeControl1 = poseFactory.of(50, 30, 0);
    private final Pose shottingPos4 = poseFactory.of(56, 9.3, 180);
    private final Pose shottingPos4Control1 = poseFactory.of(58, 32, 0);
    private final Pose point82ndTunnelIntake = poseFactory.of(9, 28, 180);
    private final Pose point82ndTunnelIntakeControl1 = poseFactory.of(50, 30, 0);
    private final Pose shootingPos5 = poseFactory.of(56, 9.3, 180);
    private final Pose shootingPos5Control1 = poseFactory.of(58, 32, 0);


    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                parallel(
                        library.hoodSetAngle(0.27),
                        library.setShooterSpeed(1900)
                ),
                race(
                        library.turretToPosition(-30),
                        waitMs(500)
                ),
                library.shootAllBalls(true),
                race(
                        library.setIntakeSpeed(2000),
                        waitMs(1)
                        ),
                follow(follower, firstBall()),
                waitMs(1500),
                follow(follower, shootingPos2()),
                race(
                        library.setIntakeSpeed(0),
                        waitMs(1)
                ),
                library.shootAllBalls(true),
                parallel(
                        race(
                                library.setIntakeSpeed(2000),
                                waitMs(1)
                        ),
                        follow(follower, path3rdSpike())
                ),

                follow(follower, endof3rdSpike()),
                follow(follower, shootingPos3()),
                race(
                        library.setIntakeSpeed(0),
                        waitMs(1)
                ),

                follow(follower, path61stTunnelIntake()),
                follow(follower, shottingPos4()),
                follow(follower, path82ndTunnelIntake()),
                follow(follower, shootingPos5())
        );
    }


    @Override
    public void runOpMode() {
        Scheduler.reset();
        robot.init();
        follower = Constants.create(hardwareMap);
        follower.setPose(start1stShootingPos);
        follower.update();

        telemetry.addData("FOLLOWER H",
                Math.toDegrees(follower.pose().heading()));

        telemetry.update();
        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();

            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            telemetry.addData("FOLLOWER heading",
                    Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Robot forward velocity", follower.twist().vx);

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path firstBall() {
        return line(start1stShootingPos, firstBall).constant(firstBall);
    }

    public Path shootingPos2() {
        return line(firstBall, shootingPos2).constant(shootingPos2);
    }

    public Path path3rdSpike() {
        return curve(shootingPos2, point3rdSpikeControl1, point3rdSpike).constant(point3rdSpike);
    }

    public Path endof3rdSpike() {
        return line(point3rdSpike, endof3rdSpike).constant(endof3rdSpike);
    }

    public Path shootingPos3() {
        return curve(endof3rdSpike, shootingPos3Control1, shootingPos3).constant(shootingPos3);
    }

    public Path path61stTunnelIntake() {
        return curve(shootingPos3, point61stTunnelIntakeControl1, point61stTunnelIntake).constant(point61stTunnelIntake);
    }

    public Path shottingPos4() {
        return curve(point61stTunnelIntake, shottingPos4Control1, shottingPos4).constant(shottingPos4);
    }

    public Path path82ndTunnelIntake() {
        return curve(shottingPos4, point82ndTunnelIntakeControl1, point82ndTunnelIntake).constant(point82ndTunnelIntake);
    }

    public Path shootingPos5() {
        return curve(point82ndTunnelIntake, shootingPos5Control1, shootingPos5).constant(shootingPos5);
    }
}



