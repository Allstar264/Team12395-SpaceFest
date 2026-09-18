package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import  org.firstinspires.ftc.teamcode.pedro.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name="Blue Pedro", group="Auto")
public class auto extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(22.5871, 123.63, 90);
    private final Pose shootingPosStart = poseFactory.of(22.5871, 123.63, 142);
    private final Pose shootingPos = poseFactory.of(47, 94.4, 130);
    private final Pose firstSpike = poseFactory.of(40, 83, 180);
    private final Pose firstSpikeControl1 = poseFactory.of(50.2757, 81.5957, 0);
    private final Pose point3 = poseFactory.of(15, 83, 180);
    private final Pose point4 = poseFactory.of(46.8729, 94.6186, 180);
    private final Pose point5 = poseFactory.of(38.1376, 59.211, 180);
    private final Pose point5Control1 = poseFactory.of(58.6756, 73.1086, 0);
    private final Pose point6 = poseFactory.of(8, 58, -177.6989);
    private final Pose point7Start = poseFactory.of(8, 58, 180);
    private final Pose point7 = poseFactory.of(47.32, 94.8114, 180);
    private final Pose point7Control1 = poseFactory.of(58.7443, 64.49, 0);
    private final Pose point8 = poseFactory.of(10, 59.81, 120);
    private final Pose point8Control1 = poseFactory.of(38.6343, 65.5514, 0);
    private final Pose point9 = poseFactory.of(47.1671, 95.8329, 180);
    private final Pose point9Control1 = poseFactory.of(47.5743, 71.0229, 0);

    public Path shootingPos() {
        return line(shootingPosStart, shootingPos).linear(shootingPosStart, shootingPos);
    }

    public Path firstSpike() {
        return curve(shootingPos, firstSpikeControl1, firstSpike).linear(shootingPos, firstSpike);
    }

    public Path path3() {
        return line(firstSpike, point3).linear(firstSpike, point3);
    }

    public Path path4() {
        return line(point3, point4).linear(point3, point4);
    }

    public Path path5() {
        return curve(point4, point5Control1, point5).linear(point4, point5);
    }

    public Path path6() {
        return line(point5, point6).tangent();
    }

    public Path path7() {
        return curve(point7Start, point7Control1, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return curve(point7, point8Control1, point8).linear(point7, point8);
    }

    public Path path9() {
        return curve(point8, point9Control1, point9).linear(point8, point9);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, shootingPos()),
                // Add mechanism commands here.
                follow(follower, firstSpike()),

                follow(follower, path3()),

                follow(follower, path4()),

                follow(follower, path5()),

                follow(follower, path6()),

                follow(follower, path7()),

                follow(follower, path8()),

                follow(follower, path9())

        );
    }


    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
    }
    @Override
    public void start() {
        schedule(autoRoutine());
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        // add your other methods needed in the loop here
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}
