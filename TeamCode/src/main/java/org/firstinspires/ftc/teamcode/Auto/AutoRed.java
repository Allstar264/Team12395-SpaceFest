package org.firstinspires.ftc.teamcode.Auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoRed extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.5, 8.6, 90);
    private final Pose pos1 = poseFactory.of(21.5, 47.3, 180);
    private final Pose pos1Control1 = poseFactory.of(49.2, 43, 0);
    private final Pose coll1 = poseFactory.of(9.8, 47.4, 180);
    private final Pose retract1 = poseFactory.of(21.1, 47.5, -179.493);
    private final Pose pos2Start = poseFactory.of(21.1, 47.5, 180);
    private final Pose pos2 = poseFactory.of(46.9, 122.3, 90);
    private final Pose pos2Control1 = poseFactory.of(17.6, 116.1, 0);
    private final Pose coll2 = poseFactory.of(46.9093, 132.0037, 90);
    private final Pose retract2 = poseFactory.of(46.6, 121.1, 90);
    private final Pose park = poseFactory.of(8.7, 99.7, 0);
    private final Pose parkControl1 = poseFactory.of(41.9432, 104.2399, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, pos1()),
                follow(follower, coll1()),
                follow(follower, retract1()),
                follow(follower, pos2()),
                follow(follower, coll2()),
                follow(follower, retract2()),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
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

    public Path pos1() {
        return curve(start, pos1Control1, pos1).linear(start, pos1);
    }

    public Path coll1() {
        return line(pos1, coll1).constant(coll1);
    }

    public Path retract1() {
        return line(coll1, retract1).reverseTangent();
    }

    public Path pos2() {
        return curve(pos2Start, pos2Control1, pos2).linear(pos2Start, pos2);
    }

    public Path coll2() {
        return line(pos2, coll2).constant(coll2);
    }

    public Path retract2() {
        return line(coll2, retract2).linear(coll2, retract2);
    }

    public Path park() {
        return curve(retract2, parkControl1, park).linear(retract2, park);
    }
}