package com.acmerobotics.roverruckus.robot;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roverruckus.util.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by ACME Robotics on 9/25/2018.
 */

public class Lift {
    public static final double LIFT_HEIGHT = 15; //find actual value in inches when lift is in CAD
    public static final double RADIUS = 1; //find actual value in inches when lift is in CAD
    public static PIDCoefficients LIFT_PID = new PIDCoefficients(0, 0, 0); //find liftPid coefficients
    MotionProfile liftProfile;

    private double startTime = 0;
    private DcMotor liftMotor1;
    private DcMotor liftMotor2;
    private PIDController pidController;
    public double startPosition;
    public double maxLiftPosition;
    public double minLiftPosition;
    private int encoderOffSet;
    private double liftPower;


    public Lift(HardwareMap hardwareMap){
        liftMotor1 = hardwareMap.dcMotor.get("LiftMotor1");
        liftMotor2 = hardwareMap.dcMotor.get("LiftMotor2");
        pidController = new PIDController(LIFT_PID);
    }


    private enum LiftMode{
        DRIVERCONTROLLED,
        HOLDPOSITION,
        RUNTOPOSITION;
    }

    private LiftMode liftMode = LiftMode.HOLDPOSITION;


    public int getEncoderPosition(){
        return liftMotor1.getCurrentPosition() + encoderOffSet;
    }

    public void setEncoderPosition(int position){
        encoderOffSet = position - liftMotor1.getCurrentPosition();
    }

    public double getStartingPosition(){
        return this.startPosition = liftMotor1.getCurrentPosition();
    }

    public void setStartingPosition(double startPos){
        this.startPosition = startPos;
    }

    public double getMaxLiftPosition(){
        return maxLiftPosition = LIFT_HEIGHT;
    }

    public void setMaxLiftPosition(double maxPos){
        maxLiftPosition = maxPos;
    }

    public double getMinLiftPosition(){
        return minLiftPosition = getStartingPosition();
    }

    public void setMinLiftPosition(double minPos){
        minLiftPosition = minPos;
    }


    public void setLiftPower(double power){
        liftPower = power;
        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
        liftMode = LiftMode.DRIVERCONTROLLED;
    }

    private int inchesToTicks(double inches) {
        double ticksPerRev = liftMotor1.getMotorType().getTicksPerRev();
        double circumference = 2 * Math.PI * RADIUS;
        return (int) Math.round(inches * ticksPerRev / circumference);
    }

    private double ticksToInches(int ticks) {
        double ticksPerRev = liftMotor1.getMotorType().getTicksPerRev();
        double revs = ticks / ticksPerRev;
        return 2 * Math.PI * RADIUS * revs;
    }

    private double getLiftHeight(){
        return ticksToInches(getEncoderPosition());
    }

    private void setLiftHeight(double height){
        setEncoderPosition(inchesToTicks(height));
    }


    public void update(){

        double liftPower;
        switch (liftMode){
            case DRIVERCONTROLLED:
                double start = getStartingPosition();
                double max = getMaxLiftPosition();
                double min = getMinLiftPosition();
                int currentPos = getEncoderPosition();
                setStartingPosition(start);
                setMaxLiftPosition(max);
                setMinLiftPosition(min);
                setEncoderPosition(currentPos);

                if(currentPos > start){
                    liftPower = this.liftPower;

                }else if(currentPos == start){
                    liftPower = this.liftPower;

                } else {
                    liftMode = LiftMode.HOLDPOSITION;
                }
                update();
                break;

            case HOLDPOSITION:
                double liftHeight = getLiftHeight();
                double error = pidController.getError(liftHeight);
                liftPower = pidController.update(error);

                break;

            case RUNTOPOSITION:
                MotionState currrentState = liftProfile.get(System.currentTimeMillis() - startTime);
                break;
        }
    }

    public void driverControlled(){
        liftMode = LiftMode.DRIVERCONTROLLED;
    }

    public void holdPosition(){
        liftMode = LiftMode.HOLDPOSITION;
    }

    public void goToPosition(double position){
        liftProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0, 0),
                new MotionState(position, 0, 0, 0),
                1, 1, 1 //find real values eventually
        );
        liftMode = LiftMode.RUNTOPOSITION;
        startTime = System.currentTimeMillis();

    }
}
