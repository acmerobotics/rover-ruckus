package com.acmerobotics.roverruckus.robot;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roverruckus.hardware.SharpDistanceSensor;
import com.acmerobotics.roverruckus.opMode.auto.Auto;
import com.acmerobotics.roverruckus.util.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

/**
 * Created by ACME Robotics on 9/25/2018.
 */

@Config
public class Lift extends Subsystem {
    public static double F_UP = .0917;
    public static double F_DOWN = .045;
    private double F = F_UP;
    public static double P = 1;
    public static double I = 0;
    public static double D = 0;
    public static double V = 14;
    public static double A = 30;
    public static double J = 30;
    public static double LOWER_P = -.5;
    public static double RADIUS = .5; //find actual value in inches when lift is in CAD
    public static double LOWER_DISTANCE = .15;
    public static double LOWER_THRESHOLD = .05;

    public static double DUMP_DOWN = .92;
    public static double DUMP_MIDDLE = .65;
    public static double DUMP_UP = .15;
    public static double RATCHET_ENGAGE = .5;
    public static double RATCHET_DISENGAGE = .85;
    public static double MARKER_UP = .6;
    public static double MARKER_DOWN = .3;
    public static double LIFT_LATCH = 14.5;
    public static double LIFT_SCORE = 20.5;
    public static double LIFT_MAX = 20.5;
    public static double LIFT_E_DUMP = 9;
    public static double LIFT_FIND_LATCH_START = 11;
    public static double LIFT_CLEARANCE = 10.25;
    public static double LIFT_DOWN = 1;

    public static double FIND_LATCH_V = .3;

    public static double K_STATIC = -.3;
    public static double CONTACT_DISTANCE = .4;
    public static int LOWER_WAIT_TIME = 1000;
    private DcMotorEx motor1, motor2;
    private Servo marker, ratchet, dump;
    private SharpDistanceSensor distance;
    private DigitalChannel liftSensor;
    private boolean ratchetEngaged = true;

    private double offset;

    private PIDController pidController;

    private long lowerStartTime;

    private MotionProfile profile;
    private double startTime;

    private double targetPosition;

    private double dumpPositionOnCompletion = DUMP_MIDDLE;
    private boolean moveDumpOnCompletion = false;
    private boolean closePlacerOnCompletion = false;
    private boolean openGateOnCompletion = false;
    private boolean emergencyDumpOnCompletion = false;
    private boolean findLatchOnCompletion = false;
    private boolean asynch = false;

    private enum LiftMode {
        RUN_TO_POSITION,
        DRIVER_CONTROLLED,
        HOLD_POSITION,
        LOWERING,
        FIND_LATCH
    }

    private LiftMode liftMode = LiftMode.DRIVER_CONTROLLED;

    public Placer placer;
    private boolean gateArmClosed = false;

    private Robot robot;

    public Lift(Robot robot, HardwareMap hardwareMap) {
        this.robot = robot;
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
        liftSensor = hardwareMap.digitalChannel.get("liftSensor");

        pidController = new PIDController(P, I, D);

        placer = new Placer(hardwareMap);

        markerDown();
        engageRatchet();
        dumpDown();
        placer.reset();
    }

    public double getPosition() {
        return ((motor1.getCurrentPosition() / motor1.getMotorType().getTicksPerRev()) * Math.PI * RADIUS * 2) + offset;
    }

    public void setPosition(double position) {
        offset = -(getPosition() - offset) + position;
    }

    @Override
    protected void update(TelemetryPacket packet) {
        packet.put("lift mode", liftMode.toString());
        packet.put("position", getPosition());
        packet.put("ratchet", ratchetEngaged);
        packet.put("distance", distance.getUnscaledDistance());


        switch (liftMode) {
            case RUN_TO_POSITION:
                double t = (System.currentTimeMillis() - startTime) / 1000.0;
                if (!gateArmClosed && getPosition() > LIFT_LATCH) {
                    placer.closeArm();
                    gateArmClosed = true;
                }
                else if (getPosition() < LIFT_LATCH) {
                    placer.openArm();
                }
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
                packet.put("liftCorrection", -correction);
                break;
            case LOWERING:
                error = distance.getUnscaledDistance() - LOWER_DISTANCE;
                packet.put("error", error);
                if (Math.abs(error) < LOWER_THRESHOLD) {
                    findLatch();
//                    liftMode = LiftMode.HOLD_POSITION;
//                    internalSetVelocity(0);
//                    targetPosition = getPosition();
//                    pidController = new PIDController(P, I, D);
                    break;
                }
                correction = pidController.update(error);
                if (System.currentTimeMillis() < lowerStartTime + LOWER_WAIT_TIME)
                    internalSetVelocity(K_STATIC);
                else if (error > CONTACT_DISTANCE)
                    internalSetVelocity(K_STATIC - correction);
                else
                    internalSetVelocity(-correction);
                break;
            case FIND_LATCH:
                internalSetVelocity(FIND_LATCH_V);
                if (!liftSensor.getState()) {
                    Log.e(Auto.TAG, "found the sensor");
                    liftMode = LiftMode.HOLD_POSITION;
                    internalSetVelocity(0);
                    targetPosition = getPosition();
                    pidController = new PIDController(P, I, D);
                }


        }

    }

    private void setDumpOnCompletion(double position) {
        dumpPositionOnCompletion = position;
        moveDumpOnCompletion = true;
    }

    private void completionDump() {
        if (moveDumpOnCompletion) dump.setPosition(dumpPositionOnCompletion);
        if (closePlacerOnCompletion) placer.closeArm();
        if (openGateOnCompletion) placer.openIntake();
        if (emergencyDumpOnCompletion) {
            placer.releaseGold();
            placer.releaseGold();
        }
        if (findLatchOnCompletion) {
            liftMode = LiftMode.FIND_LATCH;
        }
        openGateOnCompletion = false;
        moveDumpOnCompletion = false;
        closePlacerOnCompletion = false;
        emergencyDumpOnCompletion = false;
        findLatchOnCompletion = false;
    }

    public void lower() {
        pidController = new PIDController(LOWER_P, 0, 0);
        disengageRatchet();
        dumpMiddle();
        placer.reset();
        liftMode = LiftMode.LOWERING;
        lowerStartTime = System.currentTimeMillis();
    }

    public void goToPosition(double position) {
        pidController = new PIDController(P, I, D);
        if (position > getPosition()) F = F_UP;
        else F = F_DOWN;
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getPosition(), 0, 0, 0),
                new MotionState(position, 0, 0, 0),
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
        if (!ratchetEngaged) v /= 2;
        liftMode = LiftMode.DRIVER_CONTROLLED;
        moveDumpOnCompletion = false;
        closePlacerOnCompletion = false;
        openGateOnCompletion = false;
        emergencyDumpOnCompletion = false;
        if (v <= 0 || !ratchetEngaged) internalSetVelocity(v);
    }

    private void internalSetVelocity(double v) {
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

    public void liftTop() {
        goToPosition(LIFT_SCORE);
        setDumpOnCompletion(DUMP_MIDDLE);
        closePlacerOnCompletion = true;
        gateArmClosed = false;
    }

    public void liftBottom() {
        goToPosition(LIFT_DOWN);
        setDumpOnCompletion(DUMP_DOWN);
        placer.reset();
        openGateOnCompletion = true;
    }

    public void dumpUp() {
        if (liftMode == LiftMode.RUN_TO_POSITION) setDumpOnCompletion(DUMP_UP);
        else if (getPosition() < LIFT_E_DUMP) {
            goToPosition(LIFT_E_DUMP);
            setDumpOnCompletion(DUMP_UP);
        } else dump.setPosition(DUMP_UP);
        Log.e("the lift", "I guess the dump is supposed to go up lol");
    }

    public void dumpMiddle() {
        dump.setPosition(DUMP_MIDDLE);
    }

    public void dumpDown() {
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
        return !asynch && Arrays.asList(LiftMode.RUN_TO_POSITION, LiftMode.LOWERING, LiftMode.FIND_LATCH).contains(liftMode);
    }

    public boolean isSensor() {
        return liftSensor.getState();
    }

    public void findLatch() {
        findLatchOnCompletion = true;
        goToPosition(LIFT_FIND_LATCH_START);
    }

    public void releaseMarker() {
        markerUp();
        robot.pause(1000);
        markerDown();
    }

    public void setAsynch(boolean asynch) {
        this.asynch = asynch;
    }

    public double getOffset() {
        return offset;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }
}

