
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "BangBang Flywheel", group = "Test")
public class BangBangFlywheel extends OpMode {

    // YOUR EXISTING HARDWARE CLASS
    private final hardware robot = new hardware();

    // ==========================
    // DASHBOARD SETTINGS
    // ==========================

    public static double targetVelocity = 1400;
    public static double tolerance = 20;

    // Tune experimentally at 1400 TPS.
    public static double holdingPower = 0.35;

    public static double recoveryPower = 1.0;

    // Intake is open-loop power, not RPM.
    public static double intakePower = 1.0;

    public static boolean requireReady = true;
    public static double readyTolerance = 20;

    // false = gamepad1
    // true = gamepad2
    public static boolean useGamepad2 = false;

    // ==========================
    // OPTIONAL STOPPER
    // ==========================

    // Leave false unless positions are configured.
    private static final boolean USE_STOPPER = false;

    private static final double STOPPER_OPEN = -1;
    private static final double STOPPER_CLOSED = -1;

    public static double stopperDelayMs = 120;

    // ==========================
    // STATES
    // ==========================

    private boolean enabled = false;
    private boolean ffOnly = true;
    private boolean measuring = false;
    private boolean feeding = false;

    private boolean lastCross = false;
    private boolean lastSquare = false;
    private boolean lastTriangle = false;

    // ==========================
    // MEASUREMENTS
    // ==========================

    private double velocity = 0;
    private double current = 0;
    private double output = 0;

    private double minVelocity =
            Double.POSITIVE_INFINITY;

    private double peakCurrent = 0;

    private double feedTimeMs = 0;
    private long feedStartNs = 0;

    // ==========================
    // TIMING
    // ==========================

    private long lastLoopNs = 0;
    private long lastTelemetryNs = 0;
    private long lastCurrentNs = 0;

    private double loopMs = 0;
    private double averageLoopMs = 0;
    private double maxLoopMs = 0;

    private static final long TELEMETRY_INTERVAL =
            100_000_000L; // 10 Hz

    private static final long CURRENT_INTERVAL =
            50_000_000L; // 20 Hz

    @Override
    public void init() {

        // Initialize using YOUR hardware.java
        robot.init(hardwareMap);

        robot.flyWheel.setPower(0);
        robot.intakeMotor.setPower(0);

        // Your hardware class already sets:
        // Flywheel direction = REVERSE
        // Intake direction = FORWARD
        // Flywheel FLOAT
        // Intake BRAKE

        // Override built-in PIDF velocity mode.
        // Encoder velocity can still be read.
        robot.flyWheel.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        // Optional stopper initialization
        if (USE_STOPPER) {

            if (!validStopperPositions()) {
                throw new IllegalStateException(
                        "Configure stopper positions first"
                );
            }

            robot.turretstopperServo.setPosition(
                    STOPPER_CLOSED
            );
        }

        // Enable automatic bulk caching.
        for (LynxModule hub :
                hardwareMap.getAll(LynxModule.class)) {

            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.AUTO
            );
        }

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        telemetry.addLine("PS5 CONTROLS");
        telemetry.addLine("CROSS: Flywheel ON/OFF");
        telemetry.addLine("CIRCLE: Stop everything");
        telemetry.addLine("SQUARE: Switch FF / BangBang");
        telemetry.addLine("TRIANGLE: Reset measurements");
        telemetry.addLine("R1: Hold to feed balls");
        telemetry.update();
    }

    private boolean validStopperPositions() {

        return STOPPER_OPEN >= 0
                && STOPPER_OPEN <= 1
                && STOPPER_CLOSED >= 0
                && STOPPER_CLOSED <= 1;
    }

    private void closeStopper() {

        if (USE_STOPPER) {
            robot.turretstopperServo.setPosition(
                    STOPPER_CLOSED
            );
        }
    }

    @Override
    public void loop() {

        long now = System.nanoTime();

        // ==========================
        // LOOP SPEED MEASUREMENT
        // ==========================

        if (lastLoopNs != 0) {

            loopMs = (now - lastLoopNs)
                    / 1_000_000.0;

            if (averageLoopMs == 0) {
                averageLoopMs = loopMs;
            } else {
                averageLoopMs =
                        0.9 * averageLoopMs
                                + 0.1 * loopMs;
            }

            maxLoopMs = Math.max(
                    maxLoopMs, loopMs
            );
        }

        lastLoopNs = now;

        // ==========================
        // PS5 CONTROLS
        // ==========================

        Gamepad pad = useGamepad2
                ? gamepad2
                : gamepad1;

        boolean cross = pad.a;
        boolean circle = pad.b;
        boolean square = pad.x;
        boolean triangle = pad.y;
        boolean r1 = pad.right_bumper;

        // CROSS: Toggle flywheel
        if (cross && !lastCross && !circle) {
            enabled = !enabled;
            measuring = false;
        }

        // CIRCLE: Stop everything
        if (circle) {
            enabled = false;
            measuring = false;
        }

        // SQUARE: Switch controller
        if (square && !lastSquare) {
            ffOnly = !ffOnly;
            measuring = false;
        }

        // TRIANGLE: Reset shot measurements
        if (triangle && !lastTriangle) {

            minVelocity =
                    Double.POSITIVE_INFINITY;

            peakCurrent = 0;
            maxLoopMs = 0;

            measuring = enabled;
        }

        lastCross = cross;
        lastSquare = square;
        lastTriangle = triangle;

        // ==========================
        // FLYWHEEL VELOCITY
        // ==========================

        velocity = Math.abs(
                robot.flyWheel.getVelocity()
        );

        double target = Math.max(
                0, targetVelocity
        );

        double band = Math.max(
                0, tolerance
        );

        // ==========================
        // FEEDFORWARD
        // ==========================

        // Holding power calibrated at 1400 TPS.
        double kF = Range.clip(
                holdingPower, 0, 1
        ) / 1400.0;

        double feedforward = Range.clip(
                kF * target, 0, 1
        );

        // ==========================
        // BANG-BANG + FEEDFORWARD
        // ==========================

        if (!enabled || circle || target <= 0) {

            output = 0;

        } else if (ffOnly) {

            // Feedforward-only tuning
            output = feedforward;

        } else if (velocity < target - band) {

            // Full recovery
            output = recoveryPower;

        } else if (velocity > target + band) {

            // Coast
            output = 0;

        } else {

            // Maintain velocity near target
            output = feedforward;
        }

        output = Range.clip(output, 0, 1);

        // Update flywheel every loop
        robot.flyWheel.setPower(output);

        // ==========================
        // R1 INTAKE CONTROL
        // ==========================

        boolean feedRequested =
                r1 && enabled && !circle
                        && target > 0;

        boolean flywheelReady =
                velocity >= target
                        - Math.max(0, readyTolerance);

        // Begin feed only when ready.
        if (feedRequested && !feeding) {

            if (!requireReady || flywheelReady) {

                feeding = true;
                feedStartNs = now;
                feedTimeMs = 0;

                if (USE_STOPPER) {
                    robot.turretstopperServo.setPosition(
                            STOPPER_OPEN
                    );
                }
            }
        }

        if (feeding && feedRequested) {

            double elapsedMs =
                    (now - feedStartNs)
                            / 1_000_000.0;

            boolean gateReady =
                    !USE_STOPPER
                            || elapsedMs >=
                            Math.max(0, stopperDelayMs);

            if (gateReady) {

                robot.intakeMotor.setPower(
                        Range.clip(intakePower, -1, 1)
                );

                feedTimeMs = USE_STOPPER
                        ? Math.max(
                        0, elapsedMs - stopperDelayMs
                )
                        : elapsedMs;

            } else {

                robot.intakeMotor.setPower(0);
            }

        } else {

            robot.intakeMotor.setPower(0);

            if (feeding) {
                closeStopper();
            }

            feeding = false;
        }

        // ==========================
        // VELOCITY DROP MEASUREMENT
        // ==========================

        if (enabled && measuring) {

            minVelocity = Math.min(
                    minVelocity, velocity
            );
        }

        // ==========================
        // MOTOR CURRENT (20 HZ)
        // ==========================

        if (now - lastCurrentNs >=
                CURRENT_INTERVAL) {

            lastCurrentNs = now;

            current = robot.flyWheel.getCurrent(
                    CurrentUnit.AMPS
            );

            if (enabled && measuring) {

                peakCurrent = Math.max(
                        peakCurrent, current
                );
            }
        }

        // ==========================
        // TELEMETRY (10 HZ)
        // ==========================

        if (now - lastTelemetryNs >=
                TELEMETRY_INTERVAL) {

            lastTelemetryNs = now;

            telemetry.addData(
                    "Active Gamepad",
                    useGamepad2 ? 2 : 1
            );

            telemetry.addData(
                    "Flywheel Enabled", enabled
            );

            telemetry.addData(
                    "Controller",
                    ffOnly ? "FF ONLY" : "BANG-BANG + FF"
            );

            telemetry.addData(
                    "Target TPS", target
            );

            telemetry.addData(
                    "Actual TPS", velocity
            );

            telemetry.addData(
                    "Flywheel RPM",
                    velocity * 60.0 / 28.0
            );

            telemetry.addData(
                    "Velocity Error",
                    target - velocity
            );

            telemetry.addData(
                    "Motor Power", output
            );

            telemetry.addData(
                    "Feedforward Power",
                    feedforward
            );

            telemetry.addData(
                    "Motor Current A", current
            );

            telemetry.addData(
                    "Peak Current A", peakCurrent
            );

            telemetry.addData(
                    "Minimum TPS",
                    Double.isInfinite(minVelocity)
                            ? 0 : minVelocity
            );

            telemetry.addData(
                    "Measuring", measuring
            );

            // INTAKE
            telemetry.addData("R1 Pressed", r1);
            telemetry.addData("Flywheel Ready", flywheelReady);
            telemetry.addData("Feeding", feeding);

            telemetry.addData(
                    "Feed Command Time ms", feedTimeMs
            );

            // GAMEPAD DIAGNOSTICS
            telemetry.addData("Cross", cross);
            telemetry.addData("Circle", circle);
            telemetry.addData("Square", square);

            telemetry.addData(
                    "Gamepad 1 R1",
                    gamepad1.right_bumper
            );

            telemetry.addData(
                    "Gamepad 2 R1",
                    gamepad2.right_bumper
            );

            // LOOP PERFORMANCE
            telemetry.addData(
                    "Loop Time ms", loopMs
            );

            telemetry.addData(
                    "Average Loop ms",
                    averageLoopMs
            );

            telemetry.addData(
                    "Maximum Loop ms",
                    maxLoopMs
            );

            telemetry.addData(
                    "Loop Hz",
                    averageLoopMs > 0
                            ? 1000.0 / averageLoopMs
                            : 0
            );

            telemetry.update();
        }
    }

    @Override
    public void stop() {

        enabled = false;
        measuring = false;
        feeding = false;

        robot.flyWheel.setPower(0);
        robot.intakeMotor.setPower(0);

        closeStopper();
    }
}
