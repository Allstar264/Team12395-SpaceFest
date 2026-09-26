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

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.25);

    private final Pose start = poseFactory.of(122.5, 119, 36);
    private final Pose shootingPos = poseFactory.of(94.5, 94.4, 0);
    private final Pose firstSpike = poseFactory.of(98, 82.3, 0);
    private final Pose firstSpikeControl1 = poseFactory.of(91.2243, 81.5957, 0);
    private final Pose end1stSpike = poseFactory.of(122.5, 82.3, 0);
    private final Pose shootingPos2 = poseFactory.of(94.6271, 94.6186, 0);
    private final Pose secondSpike = poseFactory.of(97.5, 58, 0);
    private final Pose secondSpikeControl1 = poseFactory.of(82.8244, 73.1086, 0);
    private final Pose end2ndSpike = poseFactory.of(128, 57, 0);
    private final Pose shootingPos3 = poseFactory.of(94.18, 94.8114, 0);
    private final Pose shootingPos3Control1 = poseFactory.of(74.981, 63.1942, 0);
    private final Pose gate = poseFactory.of(129, 57.75, 25);
    private final Pose gateControl1 = poseFactory.of(75.80016722408027, 65.5514, 0);
    private final Pose shootingPos4 = poseFactory.of(94.5, 94.5, 0);
    private final Pose shootingPos4Control1 = poseFactory.of(93.9257, 71.0229, 0);

    private final Pose gate2 = poseFactory.of(129, 57.75, 25);
    private final Pose gate2Control1 = poseFactory.of(75, 65.5514, 0);
    private final Pose shootingPos5 = poseFactory.of(94.5, 94.5, 0);
    private final Pose shootingPos5Control1 = poseFactory.of(80, 71.0229, 0);
    private final Pose leave = poseFactory.of(94.5,70,0);





    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                parallel(
                        library.setShooterSpeed(1400),
                        follow(follower, shootingPos()),
                        library.hoodSetAngle(.70),
                        sequential(
                                race(
                                        library.turretToPosition(60),
                                        waitMs(500)
                                ),
                                library.turretStop()
                        )


                ),
                waitMs(250),
                library.shootAllBalls(false),
                parallel(
                        follow(follower, secondSpike()),
                        library.hoodSetAngle(.75),
                        library.zerospindexer()
                ),
                race(
                        library.setIntakeSpeed(2000),
                        waitMs(1)
                ),
                follow(follower, end2ndSpike()),
                follow(follower, shootingPos3()),
                race(
                        library.setIntakeSpeed(0),
                        waitMs(1)
                ),
                library.adjustspindexer(),
                library.shootAllBalls(false),
                parallel(
                        follow(follower, gate()),
                        library.zerospindexer(),
                        race(
                                library.setIntakeSpeed(2000),
                                waitMs(1)
                        )
                ),
                waitMs(1500),
                follow(follower, shootingPos4()),
                race(
                        library.setIntakeSpeed(0),
                        waitMs(1)
                ),
                library.adjustspindexer(),
                library.shootAllBalls(false),
                parallel(
                        follow(follower, gate2()),
                        library.zerospindexer(),
                        race(
                                library.setIntakeSpeed(2000),
                                waitMs(1)
                        )
                ),
                waitMs(1500),
                parallel(
                        follow(follower, shootingPos5()),
                        library.adjustspindexer()
                ),
                library.shootAllBalls(false),
                parallel(
                        follow(follower, firstSpike()),
                        race(
                                library.setIntakeSpeed(2000),
                                waitMs(1),
                                library.zerospindexer()
                        )
                ),
                follow(follower, end1stSpike()),
                follow(follower, shootingPos2()),
                race(
                        library.setIntakeSpeed(0),
                        waitMs(1)
                ),
                library.adjustspindexer(),
                waitMs(50),
                library.shootAllBalls(false),
                parallel(
                        follow(follower, finish()),
                        sequential(
                                race(
                                        library.turretToPosition(0),
                                        waitMs(250)
                                ),
                                library.turretStop()
                        )
                )




        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        robot.init();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        telemetry.addData("START X", start.x());
        telemetry.addData("START Y", start.y());
        telemetry.addData("START H", Math.toDegrees(start.heading()));

        telemetry.addData("TARGET X", shootingPos.x());
        telemetry.addData("TARGET Y", shootingPos.y());
        telemetry.addData("TARGET H", Math.toDegrees(shootingPos.heading()));

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
            telemetry.addData("START heading",
                    Math.toDegrees(start.heading()));

            telemetry.addData("TARGET heading",
                    Math.toDegrees(shootingPos.heading()));

            telemetry.addData("Actual Y", follower.pose().y());
            telemetry.addData("Actual X", follower.pose().x());
            telemetry.addData("distance to end", follower.distanceToEndpoint());
            telemetry.addData("is busy", follower.isBusy());

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

    public Path shootingPos() {
        return line(start, shootingPos).linear(start, shootingPos);
    }

    public Path secondSpike() {
        return curve(shootingPos, secondSpikeControl1, secondSpike).linear(shootingPos, secondSpike);
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
    }

    public Path gate2() {
        return curve(shootingPos4, gate2Control1, gate2).linear(shootingPos4, gate2);
    }

    public Path shootingPos5() {
        return curve(gate2, shootingPos5Control1, shootingPos5).linear(gate2, shootingPos5);
    }

    public Path firstSpike() {
        return curve(shootingPos5, firstSpikeControl1, firstSpike).linear(shootingPos5, firstSpike);
    }

    public Path end1stSpike() {
        return line(firstSpike, end1stSpike).linear(firstSpike, end1stSpike);
    }

    public Path shootingPos2() {
        return line(end1stSpike, shootingPos2).linear(end1stSpike, shootingPos2);
    }

    public Path finish() {
        return line(shootingPos5, leave).constant(shootingPos5);
    }
}




