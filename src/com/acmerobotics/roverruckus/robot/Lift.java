package com.acmerobotics.roverruckus.robot;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roverruckus.hardware.SharpDistanceSensor;
import com.acmerobotics.roverruckus.util.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

/**
 * Created by ACME Robotics on 9/25/2018.
 *
 */

@Config
public class Lift extends Subsystem{
    public static double F_UP = .0917;
    public static double F_DOWN = .045;
    private double F = F_UP;
    public static double P = .5;
    public static double I = 0;
    public static double D = 0;
    public static double V = 10;
    public static double A = 10;
    public static double J = 10;
    public static double LOWER_P = -.4;
    public static double RADIUS = .5; //find actual value in inches when lift is in CAD
    public static double LOWER_DISTANCE = .13;
    public static double LOWER_THRESHOLD = .01;

    public static double DUMP_DOWN = .92;
    public static double DUMP_MIDDLE = .65;
    public static double DUMP_UP = .15;
    public static double RATCHET_ENGAGE = .5;
    public static double RATCHET_DISENGAGE = .85;
    public static double MARKER_UP = .15;
    public static double MARKER_DOWN = .9;
    public static double LIFT_LATCH = 14.5;
    public static double LIFT_SCORE = 23.5;
    public static double LIFT_MAX = 23.5;

    public static double K_STATIC = -.3;
    public static double CONTACT_DISTANCE = .2;
    public static int LOWER_WAIT_TIME = 1000;
    private DcMotorEx motor1, motor2;
    private Servo marker, ratchet, dump;
    private SharpDistanceSensor distance;
    private boolean ratchetEngaged = true;

    private double offset;

    private PIDController pidController;

    private long lowerStartTime;

    private MotionProfile profile;
    private double startTime;

    private double targetPosition;

    private double dumpPositionOnCompletion = DUMP_MIDDLE;
    private boolean moveDumpOnCompletion = false;

    private enum LiftMode{
        RUN_TO_POSITION,
        DRIVER_CONTROLLED,
        HOLD_POSITION,
        LOWERING
    }

    private LiftMode liftMode = LiftMode.DRIVER_CONTROLLED;

    public Lift(Robot robot, HardwareMap hardwareMap){
        motor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setPosition(0);

        ratchet = hardwareMap.get(Servo.class, "ratchet");
        dump = hardwareMap.get(Servo.class, "dump");
        marker = hardwareMap.get(Servo.class, "marker");
        distance = new SharpDistanceSensor(hardwareMap.analogInput.get("dist"));

        pidController = new PIDController(P, I, D);

        markerUp();
        engageRatchet();
        dumpMiddle();
    }

    public double getPosition() {
        return ((motor1.getCurrentPosition() / motor1.getMotorType().getTicksPerRev()) * Math.PI * RADIUS * 2) + offset;
    }

    public void setPosition(double position) {
        offset = -(getPosition() - offset) + position;
    }

    @Override
    protected void update(TelemetryPacket packet){
        packet.put("lift mode", liftMode.toString());
        packet.put("position", getPosition());
        packet.put("ratchet", ratchetEngaged);
        packet.put("distance", distance.getUnscaledDistance());


        switch (liftMode) {
            case RUN_TO_POSITION:
                double t = (System.currentTimeMillis() - startTime) / 1000.0;
                if (t > profile.duration()) {
                    packet.put("complete", true);
                    liftMode = LiftMode.HOLD_POSITION;
                    targetPosition = profile.end().getX();
                    completionDump();
                    return;
                }
                MotionState target = profile.get(t);
                double error = getPosition() - target.getX();
                packet.put("error", error);
                double correction = pidController.update(error);
                double feedForward = F * target.getV();
                internalSetVelocity(feedForward - correction);
                break;
            case HOLD_POSITION:
                error = getPosition() - targetPosition;
                packet.put("error", error);
                correction = pidController.update(error);
                internalSetVelocity(-correction);
                break;
            case LOWERING:
                error = distance.getUnscaledDistance() - LOWER_DISTANCE;
                packet.put("error", error);
                if (Math.abs(error) < LOWER_THRESHOLD) {
                    liftMode = LiftMode.HOLD_POSITION;
                    targetPosition = getPosition();
                    pidController = new PIDController(P, I, D);
                    break;
                }
                correction = pidController.update(error);
                if (System.currentTimeMillis() < lowerStartTime + LOWER_WAIT_TIME)
                    internalSetVelocity(K_STATIC);
                else if (error > CONTACT_DISTANCE)
                    internalSetVelocity(K_STATIC - correction);
                else
                    internalSetVelocity(-correction);

        }

    }

    private void setDumpOnCompletion (double position) {
        dumpPositionOnCompletion = position;
        moveDumpOnCompletion = true;
    }

    private void completionDump () {
        if (moveDumpOnCompletion) dump.setPosition(dumpPositionOnCompletion);
        moveDumpOnCompletion = false;
    }

    public void lower() {
        pidController = new PIDController(LOWER_P, 0, 0);
        disengageRatchet();
        dumpMiddle();
        liftMode = LiftMode.LOWERING;
        lowerStartTime = System.currentTimeMillis();
    }

    public void goToPosition (double position) {
        pidController = new PIDController(P, I, D);
        if (position > getPosition()) F = F_UP;
        else F = F_DOWN;
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getPosition(), 0 ,0 ,0),
                new MotionState(position, 0 ,0 ,0),
                V, A, J
        );
        startTime = System.currentTimeMillis();
        disengageRatchet();
        liftMode = LiftMode.RUN_TO_POSITION;
    }


    public void setVelocity(double v) {
        if (Math.abs(v) < .1 && liftMode != LiftMode.DRIVER_CONTROLLED) return;
        if (getPosition() >= LIFT_MAX && v > 0) return;
        if (getPosition() <= 0 && v < 0) return;
        liftMode = LiftMode.DRIVER_CONTROLLED;
        moveDumpOnCompletion = false;
        if (v <= 0 || !ratchetEngaged) internalSetVelocity(v);
    }

    private void internalSetVelocity (double v) {
        if (v != 0 && liftMode != LiftMode.HOLD_POSITION) dumpMiddle();
        motor1.setPower(v);
        motor2.setPower(v);
    }


    public void disengageRatchet() {
        ratchet.setPosition(RATCHET_DISENGAGE);
        ratchetEngaged = false;
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void engageRatchet() {
        ratchet.setPosition(RATCHET_ENGAGE);
        ratchetEngaged = true;
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        internalSetVelocity(0);
    }

    public void liftTop () {
        goToPosition(LIFT_SCORE);
        setDumpOnCompletion(DUMP_MIDDLE);
    }

    public void liftBottom () {
        goToPosition(0);
        setDumpOnCompletion(DUMP_DOWN);
    }

    public void dumpUp () {
        dump.setPosition(DUMP_UP);
        Log.e("the lift", "I guess the dump is supposed to go up lol");
    }

    public void dumpMiddle () {
        dump.setPosition(DUMP_MIDDLE);
    }

    public void dumpDown () {
        dump.setPosition(DUMP_DOWN);
    }

    public void markerUp() {
        marker.setPosition(MARKER_UP);
    }

    public void markerDown() {
        marker.setPosition(MARKER_DOWN);
    }

    @Override
    public boolean isBusy() {
            return Arrays.asList(LiftMode.RUN_TO_POSITION, LiftMode.LOWERING).contains(liftMode);
    }
}
