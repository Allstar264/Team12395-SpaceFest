package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Reset Turret", group = "Op1")
public class resetTurret extends LinearOpMode {

    public hardware robot = new hardware();

    double stepsPerRotation = 145.090909091 * (158.0 / 26.0);
    double targetPosition = (stepsPerRotation / 360.0);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        while (opModeInInit()) {
            telemetry.addData("Angle: ", robot.turret.getCurrentPosition() * targetPosition);
            telemetry.update();
        }

        waitForStart();

        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}