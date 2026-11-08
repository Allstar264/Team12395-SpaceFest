package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Flywheel PIDF Tuner", group = "TeleOp")
@Config("Flywheel PIDF Tuner")
public class ShooterTuner extends LinearOpMode {

    private static final DcMotor.RunMode VELOCITY_MODE =
            DcMotor.RunMode.RUN_USING_ENCODER;

    public static volatile double targetVel = 700.0;  // Encoder ticks/s
    public static volatile double targetVel2 = 400.0; // Encoder ticks/s
    public static volatile double secondsPerTarget = 4.0;
    public static volatile boolean alternateTargets = true;

    // -1 loads the motor controller's current value at INIT.
    public static volatile double P = -1.0;
    public static volatile double I = -1.0;
    public static volatile double D = -1.0;
    public static volatile double F = -1.0;

    @Override
    public void runOpMode() {
        hardware robot = new hardware();
        robot.init(hardwareMap);

        DcMotorEx flywheel = robot.flyWheel;
        flywheel.setMode(VELOCITY_MODE);
        flywheel.setPower(0);

        PIDFCoefficients hubPIDF = flywheel.getPIDFCoefficients(VELOCITY_MODE);
        PIDFCoefficients lastRequested = hubPIDF;

        if (P == -1.0) P = hubPIDF.p;
        if (I == -1.0) I = hubPIDF.i;
        if (D == -1.0) D = hubPIDF.d;
        if (F == -1.0) F = hubPIDF.f;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
            dashboard.updateConfig();
        }

        telemetry.addLine("Ready. Press START for the velocity step test.");
        telemetry.update();
        waitForStart();

        ElapsedTime phaseTimer = new ElapsedTime();
        boolean firstTarget = true;

        try {
            while (opModeIsActive()) {
                double phaseSeconds = Math.max(0.25, secondsPerTarget);

                if (!alternateTargets) {
                    firstTarget = true;
                    phaseTimer.reset();
                } else if (phaseTimer.seconds() >= phaseSeconds) {
                    firstTarget = !firstTarget;
                    phaseTimer.reset();
                }

                double p = P;
                double i = I;
                double d = D;
                double f = F;

                if (p != lastRequested.p || i != lastRequested.i
                        || d != lastRequested.d || f != lastRequested.f) {
                    lastRequested = new PIDFCoefficients(p, i, d, f);
                    flywheel.setPIDFCoefficients(VELOCITY_MODE, lastRequested);
                    hubPIDF = flywheel.getPIDFCoefficients(VELOCITY_MODE);
                }

                double target = firstTarget ? targetVel : targetVel2;
                flywheel.setVelocity(target);
                double actual = flywheel.getVelocity();

                telemetry.addData("Target (ticks/s)", target);
                telemetry.addData("Actual (ticks/s)", actual);
                telemetry.addData("Seconds per target", phaseSeconds);
                telemetry.addData("Phase", firstTarget ? 1 : 2);
                telemetry.addData("Requested PIDF", "%.4f, %.4f, %.4f, %.4f",
                        p, i, d, f);
                telemetry.addData("Hub PIDF", "%.4f, %.4f, %.4f, %.4f",
                        hubPIDF.p, hubPIDF.i, hubPIDF.d, hubPIDF.f);
                telemetry.update();

                sleep(25);
            }
        } finally {
            flywheel.setPower(0);
        }
    }
}