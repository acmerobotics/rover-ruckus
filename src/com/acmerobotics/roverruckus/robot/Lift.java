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

/**
 * Created by ACME Robotics on 9/25/2018.
 *
 */

@Config
public class Lift extends Subsystem{
    public static double RADIUS = 1; //find actual value in inches when lift is in CAD
    public static double LOWER_DISTANCE = 5;
    public static double CLIMB_END_HEIGHT = 0;
    public static double CLIMB_START_HEIGHT = 4;
    public static double LIFT_HEIGHT = 30;
    public static double maxV = 1;
    public static double maxA = 1;
    public static double maxJ = 1;
    public static double lowerVelocity = 1;
    public static double climbVelocity = -1;
    public static double calibrateVelocity = -1;

    public static long WAIT_RELEASE = 1000;
    public static long WAIT_CLIMB = 1000;

    public static double LATCH_ENGAGE = 0;
    public static double LATCH_DISENGAGE = 0;
    public static double RATCHET_ENGAGE = 0;
    public static double RATCHET_DISENGAGE = 0;
    public static double GOLD_RETRACT = 0;
    public static double GOLD_EXTEND = 0;
    public static double SILVER_RETRACT = 0;
    public static double SILVER_EXTEND = 0;

    private CachingDcMotorEx motor1, motor2;
    private CachingServo ratchet, gold, silver, latch;
    private DigitalChannel liftHallEffectSensor;
    private DistanceSensor distanceSensor;

    private double offset;
    private double targetPosition;

    private MotionProfile profile;
    private long startTime;
    private PIDController pidController;
    private PIDCoefficients coefficients;

    private long waitTime = 0;

    private enum LiftMode{
        LATCHED,
        LOWERING,
        RELEASING,
        CALIBRATING,
        CLIMBING,
        HOLD_POSITION,
        RUN_TO_POSITION,
        DRIVER_CONTROLLED
    }

    private LiftMode liftMode = LiftMode.LATCHED;

    public Lift(Robot robot, HardwareMap hardwareMap){
        motor1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "LiftMotor1"));
        robot.addMotor(motor1);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "LiftMotor2"));
        robot.addMotor(motor2);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        setPosition(0);
        holdPosition();
        engageRatchet();
        coefficients = new PIDCoefficients(1, 0, 0);

        ratchet = new CachingServo(hardwareMap.get(Servo.class, "ratchet"));
        robot.addMotor(ratchet);
        gold = new CachingServo(hardwareMap.get(Servo.class, "gold"));
        robot.addMotor(gold);
        silver = new CachingServo(hardwareMap.get(Servo.class, "silver"));
        robot.addMotor(silver);
        latch = new CachingServo(hardwareMap.get(Servo.class, "latch"));


    }

    public double getPosition() {
        return ((motor1.getCurrentPosition() + offset) / motor1.getMotorType().getTicksPerRev()) * Math.PI * RADIUS * 2;
    }

    private void setPosition(double position) {
        offset = -getPosition() + position;
    }

    @Override
    public void update(TelemetryPacket packet){
        packet.put("lift mode", liftMode.toString());
        packet.put("position", getPosition());
        packet.put("limit hit", limitSensed());
        switch (liftMode) {
            case RUN_TO_POSITION:
                double t = (System.currentTimeMillis() - startTime) / 1000.0;
                MotionState target = profile.get(t);
                double error = getPosition() - target.getX();
                double correction = pidController.update(error);
                internalSetVelocity(target.getV() + correction);
                break;
            case HOLD_POSITION:
                error = getPosition() - targetPosition;
                internalSetVelocity(pidController.update(error));
                break;
            case LOWERING:
                disengageRatchet();
                double distance = distanceSensor.getDistance(DistanceUnit.CM) - LOWER_DISTANCE;
                packet.put("liftDistanceLowering", distance);
                internalSetVelocity(distance * lowerVelocity);
                if (distance == 0) {
                    waitTime = System.currentTimeMillis() + WAIT_RELEASE;
                    liftMode = LiftMode.RELEASING;
                }
                break;
            case RELEASING:
                if (System.currentTimeMillis() < waitTime) break;
                unlatch();
                liftMode = LiftMode.CALIBRATING;
                break;
            case CALIBRATING:
                internalSetVelocity(calibrateVelocity);
                if (limitSensed()) {
                    setPosition(0);
                    holdPosition();
                }
                break;
            case CLIMBING:
                internalSetVelocity(climbVelocity);
                if (getPosition() <= CLIMB_END_HEIGHT) latch();


        }
    }

    private void holdPosition(){
        disengageRatchet();
        targetPosition = getPosition();
        liftMode = LiftMode.HOLD_POSITION;
        pidController.reset();
    }

    private void goToPosition(double position){
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getPosition(), 0, 0, 0),
                new MotionState(position, 0, 0, 0),
                maxV, maxA, maxJ);
        startTime = System.currentTimeMillis();
        liftMode = LiftMode.RUN_TO_POSITION;
    }

    public void setVelocty (double v) {
        internalSetVelocity(v);
        pidController.reset();
        liftMode = LiftMode.DRIVER_CONTROLLED;
    }

    private void internalSetVelocity (double v) {
        motor1.setVelocity(v / RADIUS, AngleUnit.RADIANS);
        motor2.setVelocity(v / RADIUS, AngleUnit.RADIANS);
    }

    private boolean limitSensed() {
        return liftHallEffectSensor.getState();
    }

    private void engageRatchet() {

    }

    private void disengageRatchet() {
        ratchet.setPosition(RATCHET_DISENGAGE);
    }

    public void latch() {
        latch.setPosition(LATCH_ENGAGE);
        liftMode = LiftMode.LATCHED;
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        internalSetVelocity(0);
    }

    public void unlatch() {
        latch.setPosition(LATCH_DISENGAGE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        holdPosition();
    }
}
