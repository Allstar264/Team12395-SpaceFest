package org.firstinspires.ftc.teamcode.Auto;

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

@Autonomous(name="Blue Pedro", group="Auto")
public class BlueSideClose extends LinearOpMode {
    private Follower follower;
    Hardware robot = new Hardware(this);
    Hardware.PedroActions library =  robot.new PedroActions();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(20.1718, 118.3967, 144);
    private final Pose shootingPosStart = poseFactory.of(20.1718, 118.3967, 144);
    private final Pose shootingPos = poseFactory.of(47, 94.4, 144);
    private final Pose firstSpike = poseFactory.of(40, 83, 180);
    private final Pose firstSpikeControl1 = poseFactory.of(50.2757, 81.5957, 0);
    private final Pose end1stSpike = poseFactory.of(16.8115, 82.5974, 180);
    private final Pose shootingPos2 = poseFactory.of(46.8729, 94.6186, 180);
    private final Pose secondSpike = poseFactory.of(38.1376, 59.211, 180);
    private final Pose secondSpikeControl1 = poseFactory.of(58.6756, 73.1086, 0);
    private final Pose end2ndSpike = poseFactory.of(13.2333, 58.2013, 180);
    private final Pose shootingPos3 = poseFactory.of(47.32, 94.8114, 180);
    private final Pose shootingPos3Control1 = poseFactory.of(58.7443, 64.49, 0);
    private final Pose gate = poseFactory.of(12.4154, 59.4074, 120);
    private final Pose gateControl1 = poseFactory.of(38.6343, 65.5514, 0);
    private final Pose shootingPos4 = poseFactory.of(47.1671, 95.8329, 180);
    private final Pose shootingPos4Control1 = poseFactory.of(47.5743, 71.0229, 0);


    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                parallel(
                library.setShooterSpeed(1200),
                follow(follower, shootingPos())
                        ),
                library.shootAllBalls(false),
                parallel(
                        follow(follower, firstSpike()),
                        race(
                        library.turretToPosition(30),
                        waitMs(500)
                        )
                        ),
                race(
                        library.setIntakeSpeed(2000),
                        waitMs(1)
                ),
                follow(follower, end1stSpike()),
                follow(follower, shootingPos2()),
                race(
                        library.setIntakeSpeed(2000),
                        waitMs(1)
                ),
                library.shootAllBalls(false),
                follow(follower, secondSpike()),
                race(
                        library.setIntakeSpeed(2000),
                        waitMs(1)
                ),
                follow(follower, end2ndSpike()),
                follow(follower, shootingPos3()),
                race(
                        library.setIntakeSpeed(2000),
                        waitMs(1)
                ),
                library.shootAllBalls(false),
                race(
                        library.setIntakeSpeed(2000),
                        waitMs(1)
                ),
                follow(follower, gate()),
                waitMs(2500),
                follow(follower, shootingPos4()),
                library.shootAllBalls(false)

        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        robot.init();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();

            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path shootingPos() {
        return line(shootingPosStart, shootingPos).linear(shootingPosStart, shootingPos);
    }

    public Path firstSpike() {
        return curve(shootingPos, firstSpikeControl1, firstSpike).linear(shootingPos, firstSpike);
    }

    public Path end1stSpike() {
        return line(firstSpike, end1stSpike).linear(firstSpike, end1stSpike);
    }

    public Path shootingPos2() {
        return line(end1stSpike, shootingPos2).linear(end1stSpike, shootingPos2);
    }

    public Path secondSpike() {
        return curve(shootingPos2, secondSpikeControl1, secondSpike).linear(shootingPos2, secondSpike);
    }

    public Path end2ndSpike() {
        return line(secondSpike, end2ndSpike).linear(secondSpike, end2ndSpike);
    }

    public Path shootingPos3() {
        return curve(end2ndSpike, shootingPos3Control1, shootingPos3).linear(end2ndSpike, shootingPos3);
    }

    public Path gate() {
        return curve(shootingPos3, gateControl1, gate).linear(shootingPos3, gate);
    }

    public Path shootingPos4() {
        return curve(gate, shootingPos4Control1, shootingPos4).linear(gate, shootingPos4);
    }}




