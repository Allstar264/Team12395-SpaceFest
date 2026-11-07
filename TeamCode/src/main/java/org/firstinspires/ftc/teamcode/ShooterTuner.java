
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name="FlyWheel Tuner", group="TeleOp")
@Config
public class ShooterTuner extends LinearOpMode {
    //TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    // NOTE: One hardware instance per OpMode keeps mapping/IMU use simple and testable
    hardware robot = new hardware();

    public static double targetVel = 700;
    public static double targetVel2 = 400;
    public static double currentVel;
    public static double cycles = 20;
    public static double P;
    public static double I;
    public static double D;
    public static double F;

    public static double TelemVel;

    double clock = 0;
    boolean slow = true;

    DcMotorEx shooter;

    @Override
    public void runOpMode(){
        // Driver inputs (range roughly [-1, 1])
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap);

        shooter = robot.flyWheel;

        P = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p;
        I = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i;
        D = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d;
        F = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f;

        shooter.setVelocity(targetVel);

        waitForStart();

        while (opModeIsActive()) {
            if (clock > cycles){
                slow = !slow;
                clock = 0;
            }

            if (slow) {
                robot.setFlyWheel((int) targetVel);
                currentVel = shooter.getVelocity();
                TelemVel = targetVel;
            } else {
                robot.setFlyWheel((int) targetVel2);
                currentVel = shooter.getVelocity();
                TelemVel = targetVel2;
            }

            shooter.setVelocityPIDFCoefficients(P, I, D, F);

            telemetry.addData("target Velocity: ", TelemVel);
            telemetry.addData("current Velocity: ", currentVel);
            telemetry.addData("PIDF: ", shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.update();

            // Pace loop-helps with readability and prevents spamming the DS
            sleep(50); // ~20 Hz;
            if (clock <= cycles) {
                clock++;
            }
        }

    }
}