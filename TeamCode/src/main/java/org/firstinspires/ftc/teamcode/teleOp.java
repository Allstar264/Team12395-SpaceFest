
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Op", group = "Op1")
public class teleOp extends OpMode {

    public hardware robot = new hardware();

    private Follower follower;

    // NEW: BANG-BANG CONTROLLER
    private FlyWheelBangBangController bangBang;

    boolean flowertoggle = false;
    boolean stoppertoggle = false;
    boolean flyhweeltoggle = false;
    boolean turretoggle = false;

    double position = 0;

    // Kept for compatibility with your other code
    public static int velocity = 0;
    public static double hoodPosition = 0;

    boolean shooterclock = false;

    public static Pose targetPoint =
            new Pose(58, 54);

    double goalx = 58;
    double goaly = 54;
    double angle = 0;
    double distanceToTarget;

    private boolean upperHiveSelected = false;

    ElapsedTime shooterclockcounter =
            new ElapsedTime();

    // Telemetry timing
    private long lastTelemetryNs = 0;
    private long lastLoopNs = 0;

    private double averageLoopMs = 0;

    @Override
    public void init() {

        robot.init(hardwareMap);

        follower = Constants.create(hardwareMap);

        follower.setPose(new Pose(5.4673626, 5.23622047, 0));

        // NEW: Initialize Bang-Bang after hardware
        bangBang = new FlyWheelBangBangController(
                robot.flyWheel
        );

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        telemetry.addLine("Bang-Bang Ready");
        telemetry.addLine("Circle: Flywheel");
        telemetry.addLine("R1: Intake Forward");
        telemetry.addLine("Gamepad 2 Triangle: Reset Min TPS");
        telemetry.update();
    }

    @Override
    public void start() {

        follower.setPose(
                OpModeStorage.autonomousEndPose
        );

        follower.update();
    }

    @Override
    public void loop() {

        long now = System.nanoTime();

        // ==========================
        // LOOP TIMING
        // ==========================

        if (lastLoopNs != 0) {

            double loopMs =
                    (now - lastLoopNs) / 1_000_000.0;

            averageLoopMs =
                    averageLoopMs == 0
                            ? loopMs
                            : 0.9 * averageLoopMs
                              + 0.1 * loopMs;
        }

        lastLoopNs = now;

        // ==========================
        // PS5 CONTROLS
        // ==========================

        if (gamepad1.squareWasPressed()) {
            flowertoggle = !flowertoggle;
        }

        if (gamepad1.circleWasPressed()) {
            flyhweeltoggle = !flyhweeltoggle;
        }

        if (gamepad1.triangleWasPressed()) {
            turretoggle = !turretoggle;
        }

        if (gamepad1.crossWasPressed()) {
            stoppertoggle = !stoppertoggle;
        }

        // NEW: Reset minimum flywheel TPS
        // Uses Gamepad 2 to preserve both
        // Gamepad 1 localization buttons.
        if (gamepad2.triangleWasPressed()) {
            bangBang.resetMinimum();
        }

        // ==========================
        // BANG-BANG FLYWHEEL
        // ==========================

        // Replaces robot.setFlyWheel()
        // Update every loop.
        bangBang.update(flyhweeltoggle);

        // ==========================
        // PEDRO FIELD-CENTRIC DRIVE
        // ==========================

        DrivePowers powers =
                ManualDrive.fieldCentric(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        follower.pose().heading()
                );

        follower.manual(powers);

        // ==========================
        // RELOCALIZATION
        // ==========================

        // RIGHT: Lower corner
        if (gamepad1.dpadRightWasPressed()) {

            Pose cornerPose = new Pose(
                    5.4673626,
                    5.23622047,
                    Math.toRadians(0)
            );

            follower.setPose(cornerPose);
        }

        // LEFT: Upper corner
        if (gamepad1.dpadLeftWasPressed()) {

            Pose cornerPose2 = new Pose(
                    5.4673626,
                    136.26378,
                    Math.toRadians(0)
            );

            follower.setPose(cornerPose2);
        }

        follower.update();

        Pose robotPose = follower.pose();

        // ==========================
        // HIVE SELECTION
        // ==========================

        if (robotPose.y() < 67) {

            upperHiveSelected = false;

        } else if (robotPose.y() > 77) {

            upperHiveSelected = true;
        }

        double selectedGoalY =
                upperHiveSelected
                        ? goaly + 36
                        : goaly;

        targetPoint = new Pose(
                goalx,
                selectedGoalY,
                0
        );

        // ==========================
        // TURRET AIM
        // ==========================

        angle = robot.turretaim(
                targetPoint.x(),
                targetPoint.y(),
                robotPose.x(),
                robotPose.y(),
                robotPose.heading()
        );

        distanceToTarget = Math.abs(
                targetPoint.distance(robotPose)
        );

        // ==========================
        // INTAKE SERVO
        // ==========================

        if (flowertoggle) {

            // Down
            robot.intakeServo.setPosition(.45);

        } else {

            // Up
            robot.intakeServo.setPosition(.18);
        }

        // ==========================
        // INTAKE MOTOR
        // ==========================

        // L1: Reverse
        // R1: Feed forward

        if (gamepad1.left_bumper) {

            robot.intakeMotor.setPower(-1);

        } else if (gamepad1.right_bumper) {

            robot.intakeMotor.setPower(1);

        } else {

            robot.intakeMotor.setPower(0);
        }

        // ==========================
        // STOPPER SERVO
        // ==========================

        if (stoppertoggle) {

            robot.turretstopperServo.setPosition(.6);

        } else {

            robot.turretstopperServo.setPosition(.3);
        }

        // ==========================
        // TURRET CONTROL
        // ==========================

        if (turretoggle) {

            robot.setTurret(angle);

        } else {

            robot.setTurret(0);
        }

        // ==========================
        // DASHBOARD TELEMETRY
        // 10 Hz to reduce loop overhead
        // ==========================

        if (now - lastTelemetryNs >=
                100_000_000L) {

            lastTelemetryNs = now;

            // BANG-BANG
            telemetry.addData(
                    "Flywheel Enabled",
                    flyhweeltoggle
            );

            telemetry.addData(
                    "Flywheel Target TPS",
                    FlyWheelBangBangController.targetTPS
            );

            telemetry.addData(
                    "Flywheel Actual TPS",
                    bangBang.getActualTPS()
            );

            telemetry.addData(
                    "Flywheel Power",
                    bangBang.getPower()
            );

            telemetry.addData(
                    "Minimum TPS",
                    bangBang.getMinimumTPS()
            );

            telemetry.addData(
                    "Shooter Current",
                    robot.flyWheel.getCurrent(
                            CurrentUnit.AMPS
                    )
            );

            telemetry.addData(
                    "Average Loop ms",
                    averageLoopMs
            );

            // INTAKE
            telemetry.addData(
                    "R1 Intake",
                    gamepad1.right_bumper
            );

            telemetry.addData(
                    "Stopper Open",
                    stoppertoggle
            );

            // LOCALIZATION
            telemetry.addData(
                    "Selected Hive",
                    upperHiveSelected
                            ? "Upper"
                            : "Lower"
            );

            telemetry.addData(
                    "Target X (in)",
                    targetPoint.x()
            );

            telemetry.addData(
                    "Target Y (in)",
                    targetPoint.y()
            );

            telemetry.addData(
                    "Robot X (in)",
                    robotPose.x()
            );

            telemetry.addData(
                    "Robot Y (in)",
                    robotPose.y()
            );

            telemetry.addData(
                    "Robot Heading (deg)",
                    Math.toDegrees(robotPose.heading())
            );

            // TURRET
            telemetry.addData(
                    "Turret Auto Aim",
                    turretoggle
            );

            telemetry.addData(
                    "Turret Position",
                    angle
            );

            telemetry.addData(
                    "Turret Target",
                    robot.turret.getTargetPosition()
            );

            telemetry.addData(
                    "Turret Actual",
                    robot.turret.getCurrentPosition()
            );

            telemetry.addData(
                    "Distance to Target",
                    distanceToTarget
            );

            telemetry.update();
        }
    }

    // ==========================
    // STOP ALL MOTORS
    // ==========================

    @Override
    public void stop() {

        flyhweeltoggle = false;

        if (bangBang != null) {
            bangBang.stop();
        }

        if (robot.intakeMotor != null) {
            robot.intakeMotor.setPower(0);
        }

        if (robot.turret != null) {
            robot.turret.setPower(0);
        }

        if (robot.frontLeftMotor != null) {
            robot.setDrivePower(0, 0, 0, 0);
        }
    }
}
