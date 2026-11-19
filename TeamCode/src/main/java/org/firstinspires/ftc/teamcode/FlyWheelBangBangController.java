
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@Config
public class FlyWheelBangBangController {

    // DASHBOARD SETTINGS
    public static double targetTPS = 1400;
    public static double holdingPower = 0.57;
    public static double toleranceTPS = 20;
    public static double recoveryPower = 1.0;

    private final DcMotorEx motor;

    private double actualTPS = 0;
    private double commandedPower = 0;

    private double minimumTPS =
            Double.POSITIVE_INFINITY;

    private boolean measuring = false;

    public FlyWheelBangBangController(DcMotorEx motor) {

        this.motor = motor;

        motor.setPower(0);

        motor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        // Use custom Bang-Bang instead of hub PIDF
        motor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );
    }

    // CALL EVERY TELEOP LOOP
    public void update(boolean enabled) {

        double target = Math.max(0, targetTPS);
        double band = Math.max(0, toleranceTPS);

        actualTPS = Math.abs(motor.getVelocity());

        // Holding power was tuned at 1400 TPS.
        // Approximate scaling for other targets.
        double feedforward = Range.clip(
                holdingPower * target / 1400.0,
                0,
                1
        );

        if (!enabled || target <= 0) {

            commandedPower = 0;

        } else if (actualTPS < target - band) {

            // Full power for recovery
            commandedPower = recoveryPower;

        } else if (actualTPS > target + band) {

            // Coast when overspeeding
            commandedPower = 0;

        } else {

            // Feedforward inside tolerance
            commandedPower = feedforward;
        }

        commandedPower = Range.clip(
                commandedPower, 0, 1
        );

        motor.setPower(commandedPower);

        // Record minimum velocity after reset
        if (enabled && measuring) {

            minimumTPS = Math.min(
                    minimumTPS, actualTPS
            );
        }
    }

    public void resetMinimum() {

        minimumTPS = Double.POSITIVE_INFINITY;
        measuring = true;
    }

    public double getActualTPS() {
        return actualTPS;
    }

    public double getPower() {
        return commandedPower;
    }

    public double getMinimumTPS() {

        return Double.isInfinite(minimumTPS)
                ? 0 : minimumTPS;
    }

    public void stop() {

        measuring = false;
        commandedPower = 0;
        motor.setPower(0);
    }
}
