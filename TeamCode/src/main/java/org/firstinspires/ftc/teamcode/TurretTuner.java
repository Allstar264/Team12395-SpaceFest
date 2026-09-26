
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static android.os.SystemClock.sleep;

@TeleOp(name="Turret Tuner", group="TeleOp")
@Config
public class TurretTuner extends OpMode {
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
    public void init(){
        // Driver inputs (range roughly [-1, 1])
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();

        turret = robot.turretModule;

        P1 = TurretModule.P1;
        I1 = TurretModule.I1;
        D1 = TurretModule.D1;
        F1 = TurretModule.F1;

        P2 = TurretModule.P2;
        I2 = TurretModule.I2;
        D2 = TurretModule.D2;
        F2 = TurretModule.F2;
    }

    @Override
    public void loop() {
        if (clock > cycles){
            slow = !slow;
            clock = 0;
        }


        turret.update();

        if (slow) {
            turret.setTargetDegrees(targetPos);
            currentPos = turret.getCurrentDegrees();
            TelemPos = targetPos;
        } else {
            turret.setTargetDegrees(targetPos2);
            currentPos = turret.getCurrentDegrees();
            TelemPos = targetPos2;
        }

        turret.update();

        telemetry.addData("target Velocity: ", TelemPos);
        telemetry.addData("current Velocity: ", currentPos);
        telemetry.update();

        //panels.addData("Target Vel", TelemVel);
        //panels.addData("Current Velocity", currentVel);

        // Pace loop-helps with readability and prevents spamming the DS
        sleep(50); // ~20 Hz;
        if (clock <= cycles) {
            clock++;
        }
    }
}