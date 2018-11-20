package com.acmerobotics.roverruckus.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.profile.MotionConstraints;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roverruckus.util.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

/**
 * Created by ACME Robotics on 9/25/2018.
 *
 */

@Config
public class Lift extends Subsystem{
    public static double RADIUS = 1; //find actual value in inches when lift is in CAD
    public static double LOWER_DISTANCE = 5;
    public static double LOWER_TOLERANCE = .1;

    public static long WAIT_RELEASE = 1000;

    public static double LATCH_ENGAGE = .15;
    public static double LATCH_DISENGAGE = .9;
    public static double RATCHET_ENGAGE = .1;
    public static double RATCHET_DISENGAGE = .5;
    public static double MARKER_UP = .15;
    public static double MARKER_DOWN = .9;

    private CachingDcMotorEx motor1, motor2;
    private CachingServo ratchet, latch, marker;

    private double offset;

    private PIDController pidController;
    public PIDCoefficients pidCoefficients = new PIDCoefficients(1, 0, 0);

    private long waitTime = 0;

    private enum LiftMode{
        LATCHED,
        LOWERING,
        RELEASING,
        CALIBRATING,
        HOLD_POSITION,
        RUN_TO_POSITION,
        DRIVER_CONTROLLED
    }

    private LiftMode liftMode = LiftMode.LATCHED;

    public Lift(Robot robot, HardwareMap hardwareMap){
        motor1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftMotor1"));
        robot.addMotor(motor1);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftMotor2"));
        robot.addMotor(motor2);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        setPosition(0);

        ratchet = new CachingServo(hardwareMap.get(Servo.class, "ratchet"));
        robot.addMotor(ratchet);
        latch = new CachingServo(hardwareMap.get(Servo.class, "latch"));
        robot.addMotor(latch);
        marker = new CachingServo(hardwareMap.get(Servo.class, "marker"));
        robot.addMotor(marker);

        pidController = new PIDController(pidCoefficients);

        engageLatchAndRatchet();
        markerUp();
    }

    private double getPosition() {
        return ((motor1.getCurrentPosition() + offset) / motor1.getMotorType().getTicksPerRev()) * Math.PI * RADIUS * 2;
    }

    private void setPosition(double position) {
        offset = -getPosition() + position;
    }

    @Override
    protected void update(TelemetryPacket packet){
        packet.put("lift mode", liftMode.toString());
        packet.put("position", getPosition());


        switch (liftMode) {
//            case LATCHED:
//                break;
//           case LOWERING:
////               disengageRatchet();
//               double error = getPosition() - LOWER_DISTANCE;
//               packet.put("liftError", error);
//               if (Math.abs(error) < LOWER_TOLERANCE) {
//                   internalSetVelocity(0);
//                   liftMode = LiftMode.RELEASING;
//                   waitTime = System.currentTimeMillis() + WAIT_RELEASE;
//                   break;
//               }
//               internalSetVelocity(pidController.update(error));
//               break;
//
//
//           case RELEASING:
////               disengageLatch();
//               if (System.currentTimeMillis() < waitTime) break;
//               break;




        }
    }


    public void setVelocty (double v) {

        internalSetVelocity(v);
        liftMode = LiftMode.DRIVER_CONTROLLED;
    }

    private void internalSetVelocity (double v) {
        motor1.setPower(v);
        motor2.setPower(v);
    }


    private void disengageRatchet() {
        ratchet.setPosition(RATCHET_DISENGAGE);
//        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void engageLatchAndRatchet() {
        latch.setPosition(LATCH_ENGAGE);
        ratchet.setPosition(RATCHET_ENGAGE);
        liftMode = LiftMode.LATCHED;
//        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        internalSetVelocity(0);
    }

    private void disengageLatch() {
        latch.setPosition(LATCH_DISENGAGE);
    }

    public void lower() {
        disengageRatchet();
        liftMode = LiftMode.LOWERING;
    }

    public void latch() {
        engageLatchAndRatchet();
        liftMode = LiftMode.LATCHED;
    }

    public void unlatch() {
        disengageRatchet();
        disengageLatch();
    }

    public void markerUp() {
        marker.setPosition(MARKER_UP);
    }

    public void markerDown() {
        marker.setPosition(MARKER_DOWN);
    }

    @Override
    public boolean isBusy() {
        return Arrays.asList(LiftMode.DRIVER_CONTROLLED, LiftMode.HOLD_POSITION, LiftMode.LATCHED).contains(liftMode);
    }
}
