package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

public class TurretModule {
    private double targetDegrees = 0;
    private double lastVelocity = 0;
    public static double errorDegreesThreshold = 20;
    private final double driveToTurretRatio = 3; // 3 rotations to 1, 120/40 teeth
    private final double turretTicksPerRevolution = driveToTurretRatio *8192;// RevCoder CPR * ratio per 1 turret rev
    public final double turretTicksPerDegree = turretTicksPerRevolution/360;
    public static int maxTurnCCW = 140;
    public static int maxTurnCW = 140;
    OverflowEncoder encoder;
    CRServo servo1, servo2;
    public static double P1 = 0.00035;
    public static double I1 = 0.00002;
    public static double D1 = 0.00004;
    public static double F1 = 0;
    PIDFCustomLoop primaryController = new PIDFCustomLoop(P1, I1, D1, F1);

    public static double P2 = 0.00035;
    public static double I2 = 0.00002;
    public static double D2 = 0.00004;
    public static double F2 = 0;
    PIDFCustomLoop secondaryController = new PIDFCustomLoop(P2, I2, D2, F2);
    PIDFCustomLoop activeController = primaryController;

    public TurretModule(OverflowEncoder encoder, CRServo servo1, CRServo servo2){
        this.encoder = encoder;
        this.servo1 = servo1;
        this.servo2 = servo2;

        primaryController.setTolerance(15);
        primaryController.setSetPoint(0);

        secondaryController.setTolerance(15);
        secondaryController.setSetPoint(0);
    }

    public void setTargetDegrees(double degrees){
        degrees = Range.clip(degrees, -maxTurnCCW, maxTurnCW);
        targetDegrees = degrees;

        double target = degrees*turretTicksPerDegree;

        primaryController.setSetPoint(target);
        secondaryController.setSetPoint(target);
    }


    public void update() {
        setActiveController();
        double output = activeController.calculate(getCurrentPosition(), getCurrentVelocity());
        if (!activeController.atSetPoint()){
            setServoPowers(output);
        } else {
            stopServos();
        }
    }


    public boolean setActiveController(){
        if (!activeController.equals(secondaryController)
                && Math.abs(getErrorDegrees()) < errorDegreesThreshold){
            activeController = secondaryController;
            return true;
        } else if (!activeController.equals(primaryController)
                && Math.abs(getErrorDegrees()) > errorDegreesThreshold){
            activeController = primaryController;
            return true;
        } else {
            return false;
        }
    }

    public double getErrorDegrees(){
        return activeController.getPositionError()/turretTicksPerDegree;
    }

    private double getCurrentPosition(){
        return encoder.getPositionAndVelocity().position;
    }

    public double getCurrentDegrees(){
        return encoder.getPositionAndVelocity().position/turretTicksPerDegree;
    }

    public double getTargetDegrees(){
        return targetDegrees;
    }

    public double getCurrentError(){
        return activeController.getPositionError();
    }

    public double getCurrentVelocity(){
        double velocity;
        double vel = encoder.getPositionAndVelocity().velocity;
        if (!Double.isNaN(vel) && Double.isFinite(vel)){
            velocity = vel;
            lastVelocity = velocity;
        } else {
            velocity = lastVelocity;
        }
        return velocity;
    }

    private void setServoPowers(double p){
        //p = Range.clip(Math.abs(p), 0, 1)*(p/Math.abs(p));
        p = Range.clip(p, -1, 1);
        servo1.setPower(p);
        servo2.setPower(p);
    }

    public void stopServos(){
        servo1.setPower(0);
        servo2.setPower(0);
    }

    public boolean isDone(){
        return activeController.atSetPoint();
    }

    public void setControllerToUse(int controller){
        if (controller == 1){
            primaryController = new PIDFCustomLoop(P1, I1, D1, F1);
            secondaryController = primaryController;
            activeController = primaryController;
            setTargetDegrees(targetDegrees);
        } else if (controller == 2){
            secondaryController = new PIDFCustomLoop(P2, I2, D2, F2);
            primaryController = secondaryController;
            activeController = secondaryController;
            setTargetDegrees(targetDegrees);
        } else if (controller == 3){
            primaryController = new PIDFCustomLoop(P1, I1, D1, F1);
            secondaryController = new PIDFCustomLoop(P2, I2, D2, F2);
            activeController = primaryController;
            setTargetDegrees(targetDegrees);
        }
    }
}
