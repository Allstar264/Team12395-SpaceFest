/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode; // TODO(STUDENTS): Change to your team package (e.g., org.firstinspires.ftc.team12345.teleop)


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name="Field Centric Re: (BLUE Solo)", group="TeleOp")
@Config
public class BlueFieldCentric extends LinearOpMode {

    // NOTE: One hardware instance per OpMode keeps mapping/IMU use simple and testable
    Hardware robot = new Hardware(this);

    public static double velocity = 0;
    public static double hoodAngle = 0.1;

    public static double slowFireRateVelocity = 1800;
    public static int normalSpinVelocity = 1300;
    public static Pose baseTargetPoint = new Pose(-65, -59);
    public static double preSetAngleClose = 0.8;
    public static double preSetVelocityClose = 1400;
    public static double intakeVel = 2600;
    public static double flyWheelConstant = 1.1;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Driver inputs (range roughly [-1, 1])
        double axial    ; // forward/back (+ forward)
        double lateral  ; // strafe left/right (+ right)
        double yaw      ; // rotation (+ CCW/left turn)

        int lastTrackingClock = 10;
        double lastTargetTurretPos = 0;
        double headingVelocity;
        double distanceToTarget;

        boolean intakeToggle = false;
        boolean turretToggle = false;
        boolean fireControlToggle = false;

        double effectiveDistanceToTarget;
        Pose effectiveTargetPoint;
        Vector worldVelocity;

        robot.init();

        robot.limelight.start();

        robot.mag = "000";

        robot.disableDriveEncoders();

        waitForStart();

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {

            axial   = -gamepad1.left_stick_y;
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;

            robot.driveFieldCentric(axial, lateral, yaw);


            headingVelocity = robot.getHeadingVelocity();
            telemetry.addData("Heading Velocity: ",  headingVelocity);
            TelemetryPacket packet = new TelemetryPacket();

            Pose llpose = robot.fetchLocalizedPose(-90);

            if (!Double.isNaN(llpose.x()) && !Double.isNaN(llpose.y())) {
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), llpose);

                if (Math.abs(headingVelocity) < 80) {
                    robot.setLocalizerPosition(llpose);
                }
            }

            worldVelocity = robot.pedroFollower.velocity().toVector();
            robot.pedroFollower.update();
            Pose currentPose = robot.pedroFollower.pose();
            distanceToTarget = baseTargetPoint.distance(currentPose);
            double linearVelocity = robot.tpsToLinearVelocityMETERS();
            telemetry.addData("Linear Veloctiy M/S: ", linearVelocity);


            if (gamepad1.xWasPressed()){
                fireControlToggle = !fireControlToggle;
            } else if (gamepad1.bWasPressed()){
                velocity = 0;
                fireControlToggle = false;
            } else if (gamepad1.aWasPressed()){
                velocity = preSetVelocityClose; hoodAngle = preSetAngleClose;
                fireControlToggle = false;
            }

            if (fireControlToggle){
                velocity  = robot.getRegressionValue(distanceToTarget, 1);
                hoodAngle = robot.getRegressionValue(distanceToTarget, 2);
                telemetry.addData("Calculated Velocity: ", velocity);
                telemetry.addData("Hood Angle: ", hoodAngle);
            }


            // THIS REMAINS IN INCHES
            // drift calculated with ideal values (velocity is not measured)
            Vector2D drift = worldVelocity.times(flyWheelConstant*robot.timeToTarget(linearVelocity, robot.hoodAngleToBallisticAngle())).toVector2D();
            // shift goal
            effectiveTargetPoint = baseTargetPoint.minus(drift.toPose());

            // -----

            double angle = -robot.turretAngleToTarget(effectiveTargetPoint, currentPose);
            telemetry.addData("Angle: ", Math.toDegrees(angle));

            if (turretToggle){
                robot.turretModule.setTargetDegrees(Math.toDegrees(angle));
            } else {
                robot.turretModule.setTargetDegrees(0);
            }
            robot.turretModule.update();


            robot.setShooterVelocity(velocity);



            if (gamepad1.yWasPressed()){
                intakeToggle = !intakeToggle;
                intakeVel = Math.abs(intakeVel);
            } else if (gamepad1.dpadDownWasPressed()){
                if (!(intakeVel < 0) || !intakeToggle){
                    intakeToggle = true;
                    intakeVel = -intakeVel;
                } else {
                    intakeToggle = false;
                }
            }

            if (intakeToggle){
                robot.setIntakeVelocity(intakeVel);
            } else {
                robot.setIntakeVelocity(0);
            }


            // gamepad 2 (g1 cuz solo)--
            if (gamepad1.leftBumperWasPressed() && (robot.spindexerTarget % 120) == 0) { // ccw
                robot.spindexerHandler(-20);
            } else if (gamepad1.rightBumperWasPressed()) { // cw
                if (velocity >= slowFireRateVelocity){
                    robot.spindexerHandler(-480 - (robot.spindexerTarget % 120), 700);
                } else {
                    robot.spindexerHandler(-480 - (robot.spindexerTarget % 120), normalSpinVelocity);
                }
                robot.setMagManualBulk("000");
            }
            if (gamepad1.dpadLeftWasPressed()){
                robot.spindexer.setPower(0);
            }


            if (gamepad1.dpadRightWasPressed()){
                turretToggle = !turretToggle;
            }

            robot.setHoodAngle(hoodAngle);

            // effective target
            packet.fieldOverlay().setStroke("#F2F527");
            Drawing.drawRobot(packet.fieldOverlay(), effectiveTargetPoint.withHeading(drift.theta()));

            // base target
            packet.fieldOverlay().setStroke("#F59527");
            Drawing.drawRobot(packet.fieldOverlay(), baseTargetPoint);

            // Odometry pose (red)
            packet.fieldOverlay().setStroke("#B53F51");
            Drawing.drawRobot(packet.fieldOverlay(), currentPose);
            // Turret angle (green)
            packet.fieldOverlay().setStroke("#008000");
            Drawing.drawRobot(packet.fieldOverlay(), currentPose.withHeading(currentPose.heading() - angle));

            FtcDashboard.getInstance().sendTelemetryPacket(packet);


            /*
            if (Math.abs(robot.spindexerFudge % 360) >= 3 ){
                telemetry.addData("SPINDEXER CORRECTING: ", "");
                robot.maintainSpindexerHandler();
            }

             */

            if (gamepad2.a){
                robot.pattern = "GPP";
            } else if (gamepad2.b){
                robot.pattern = "PGP";
            } else if (gamepad2.y){
                robot.pattern = "PPG";
            }

            robot.getSpindexerOffset();
            robot.scanColor();

            // Telemetry for drivers + debugging
            telemetry.addData(robot.getMagPicture(), "");
            telemetry.addData("current Pattern: ", robot.pattern);
            telemetry.addData("Measured Velocity: ", robot.shooter.getVelocity());
            telemetry.addData("spindexerE heading? ", ((robot.spindexerE.getPositionAndVelocity().position/robot.spindexerETicksPerDegree) % 360));
            telemetry.addData("spindexer error: ", robot.spindexerFudge);
            telemetry.addData("Distance to Target: ", distanceToTarget);
            telemetry.addData("Compensated Angle: ", robot.compensatedHoodAngleSolve());
            if (robot.solvePattern() != null) {
                telemetry.addData("Turns To Solve: ", robot.solvePattern()[0]);
            }



            telemetry.update();

            if (lastTrackingClock <= 2000){
                lastTrackingClock++;
            }
        }

    }
}

