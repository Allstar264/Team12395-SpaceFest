package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

public class PedroActions {
    hardware h;

    public PedroActions() {
    }

    public void initialize(hardware hardware) {
        h = hardware;
    }

    public Command setshooterspeed(double velocity) {
        return Command.build()
                .setStart(() -> h.flyWheel.setVelocity(velocity))
                .setDone(() -> true);
    }

    public Command servostopper(double position) {
        return Command.build()
                .setStart(() -> h.setTurretstopperServo(position))
                .setDone(() -> true);
    }

    public Command intake(double power) {
        return Command.build()
                .setStart(() -> h.intakeMotor.setPower(power))
                .setDone(() -> true);
    }

    public Command flowerservo (double position) {
        return Command.build()
                .setStart(()-> h.intakeServo.setPosition(position))
                .setDone(() -> true);
    }

    // Autonomous hive switching:
    // Y < 67: select lower hive.
    // Y > 77: select upper hive.
    // Between 67 and 77: keep the previously selected hive.
    // Recalculate the aiming angle every loop.
    public Command trackTurretHives(
            Follower follower,
            double goalX,
            double lowerGoalY,
            double upperGoalY) {

        final boolean[] upperHiveSelected = {false};

        return Command.build()
                .setStart(() -> upperHiveSelected[0] = false)
                .setExecute(() -> {
                    Pose pose = follower.pose();

                    if (pose.y() < 67) {
                        upperHiveSelected[0] = false;
                    } else if (pose.y() > 77) {
                        upperHiveSelected[0] = true;
                    }

                    double selectedGoalY = upperHiveSelected[0]
                            ? upperGoalY
                            : lowerGoalY;

                    double angle = h.turretaim(
                            goalX,
                            selectedGoalY,
                            pose.x(),
                            pose.y(),
                            pose.heading()
                    );

                    h.setTurret(angle);
                })
                .setDone(() -> false)
                .setEnd(reason -> h.turret.setPower(0))
                .requiring(h.turret);
    }

    // Tracks one fixed target.
    public Command trackTurret(
            Follower follower,
            double goalX,
            double goalY) {

        return Command.build()
                .setExecute(() -> {
                    Pose pose = follower.pose();

                    double angle = h.turretaim(
                            goalX,
                            goalY,
                            pose.x(),
                            pose.y(),
                            pose.heading()
                    );

                    h.setTurret(angle);
                })
                .setDone(() -> false)
                .setEnd(reason -> h.turret.setPower(0))
                .requiring(h.turret);
    }
}
