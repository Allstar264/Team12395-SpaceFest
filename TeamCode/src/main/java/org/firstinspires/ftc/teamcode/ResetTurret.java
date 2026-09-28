
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static android.os.SystemClock.sleep;

@TeleOp(name="Reset Turret", group="TeleOp")
@Config
public class ResetTurret extends LinearOpMode {
    //TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    // NOTE: One hardware instance per OpMode keeps mapping/IMU use simple and testable
    Hardware robot = new Hardware(this);

    public static volatile double targetPos = 20;
    public static volatile double targetPos2 = 0;
    public static volatile double currentPos;
    public static volatile double cycles = 20;
    public static double P1;
    public static double I1;
    public static double D1;
    public static double F1;

    public static double P2;
    public static double I2;
    public static double D2;
    public static double F2;

    public static volatile double TelemPos;

    double clock = 0;
    boolean slow = true;
    boolean run = false;
    int controller = 1;

    TurretModule turret;

    @Override
    public void runOpMode(){
        // Driver inputs (range roughly [-1, 1])
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();

        turret = robot.turretModule;

        waitForStart();

        if (opModeIsActive()) {
            robot.shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}