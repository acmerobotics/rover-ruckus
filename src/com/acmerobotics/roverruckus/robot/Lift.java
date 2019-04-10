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

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Created by ACME Robotics on 9/25/2018.
 */

@Config
public class Lift extends Subsystem {
    public static double K_V = 0.0643202688898; //power / in/sec
    public static double K_STATIC = 0.118777073737;
    public static double K_A = 0.000127390384279;
    public static double MASS_CARTRIGE = .75;
    public static double MASS_FRAME = .75;
    public static double MASS_GOLD = .05;
    public static double MASS_SILVER = .03;
    public static double G = 386;
    public static double P = .5;
    public static double I = 0;
    public static double D = 0;
    public static double V = 15;
    public static double A = 30;
    public static double J = 30;
    public static double LOWER_P = -.5;
    public static double RADIUS = .5;
    public static double LOWER_DISTANCE = .15;
    public static double LOWER_THRESHOLD = .05;

    public static double DUMP_DOWN = .8;
    public static double DUMP_MIDDLE = .5;
    public static double DUMP_UP = .05;
    public static double RATCHET_ENGAGE = .3;
    public static double RATCHET_DISENGAGE = .85;

    public static double LIFT_LATCH = 14.5;
    public static double LIFT_SCORE = 26;
    public static double LIFT_MAX = 30;
    public static double LIFT_E_DUMP = 9;
    public static double LIFT_FIND_LATCH_START_BELOW = 14;
    public static double LIFT_FIND_LATCH_START_ABOVE = 20;
    public static double LIFT_CLEARANCE = 10.25;
    public static double LIFT_DOWN = .5;

    public static double CALIBRATE_V = .3;

    public static double CONTACT_DISTANCE = .4;
    public static int LOWER_WAIT_TIME = 1000;
    public static double GATE_OPEN_WAIT_TIME = 500;

    private DcMotorEx motor1, motor2;
    private Servo ratchet, dump;
    private SharpDistanceSensor distance;
    private boolean ratchetEngaged = true;

    private double offset;

    private PIDController pidController;

    private long lowerStartTime;

    private MotionProfile profile;
    private double startTime;

    private double targetPosition;
    private double findLatchDirection = 1;

    private enum LiftMode {
        RUN_TO_POSITION,
        DRIVER_CONTROLLED,
        HOLD_POSITION,
        LOWERING,
        FIND_LATCH,
        FIND_BOTTOM
    }

    private LiftMode liftMode = LiftMode.DRIVER_CONTROLLED;

    public Placer placer;
    private boolean gateArmClosed = false;
    private boolean dumped = false;
    private boolean liftRequested = false;
    private double liftTime = 0;

    private Robot robot;

    private interface CompletionAction {
        void onCompletion();
    }

    private ArrayList<CompletionAction> actionsOnComplete;

    private double dumpPositionOnCompletion = DUMP_MIDDLE;
    private final CompletionAction dumpAction = () -> dump.setPosition(dumpPositionOnCompletion);
    private final CompletionAction placerCloseAction = () -> placer.closeGate();
    private final CompletionAction resetAction = () -> placer.reset();
    private final CompletionAction eDumpAction = () -> placer.releaseGold();
    private final CompletionAction findLatchAction = () -> liftMode = LiftMode.FIND_LATCH;
    private final CompletionAction findBottomAction = () -> liftMode = LiftMode.FIND_BOTTOM;

    private boolean asynch = false;


    public Lift(Robot robot, HardwareMap hardwareMap) {
        this.robot = robot;
        motor1 = robot.getMotor("liftMotor1");
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2 = robot.getMotor("liftMotor2");
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setPosition(0);

        distance = new SharpDistanceSensor(robot.getAnalogInput(0, 0)); //todo check this port
        ratchet = hardwareMap.get(Servo.class, "ratchet");
        dump = hardwareMap.get(Servo.class, "dump");

        pidController = new PIDController(P, I, D);

        placer = new Placer(robot, hardwareMap);

        markerDown();
        engageRatchet();
        dumpDown();
        placer.reset();
        actionsOnComplete = new ArrayList<>();
    }

    public double getPosition() {
        return ((motor1.getCurrentPosition() / (motor1.getMotorType().getTicksPerRev() * (18.0 / 20.0))) * Math.PI * RADIUS * 2) + offset;
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

        placer.update(packet);

        if (liftRequested && System.currentTimeMillis() > liftTime) executeLiftTop();

        switch (liftMode) {
            case RUN_TO_POSITION:
                double t = (System.currentTimeMillis() - startTime) / 1000.0;
                if (!gateArmClosed && getPosition() > LIFT_LATCH && !placer.isBusy()) {
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
                double feedForward = getFeedForward(target, getMass());
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
                internalSetVelocity(CALIBRATE_V * findLatchDirection);
                if (isAtLatch()) {
                    liftMode = LiftMode.HOLD_POSITION;
                    internalSetVelocity(0);
                    targetPosition = getPosition();
                    pidController = new PIDController(P, I, D);
                }
                break;
            case FIND_BOTTOM:
                internalSetVelocity(-CALIBRATE_V);
                if (isAtBottom()) {
                    Log.i(Robot.TAG, "found the bottom");
                    liftMode = LiftMode.HOLD_POSITION;
                    internalSetVelocity(0);
                    targetPosition = 0;
                    setPosition(0);
                    pidController = new PIDController(P, I, D);
                }
        }

    }

    private void setDumpOnCompletion(double position) {
        dumpPositionOnCompletion = position;
        addCompletionAction(dumpAction);
    }

    private void completionDump() {
        executeCompletionActions();
    }

    public void lower() {
        pidController = new PIDController(LOWER_P, 0, 0);
        disengageRatchet();
        dumpMiddle();
        liftMode = LiftMode.LOWERING;
        lowerStartTime = System.currentTimeMillis();
        dumped = false;
    }

    public void goToPosition(double position, double v) {
        pidController = new PIDController(P, I, D);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getPosition(), 0, 0, 0),
                new MotionState(position, 0, 0, 0),
                V, A, J
        );
        startTime = System.currentTimeMillis();
        disengageRatchet();
        liftMode = LiftMode.RUN_TO_POSITION;
        resetCompletionActions();
    }

    public void goToPosition (double position) {
        goToPosition(position, 0);
    }


    public void setVelocity(double v) {
        if (Math.abs(v) < .1 && liftMode != LiftMode.DRIVER_CONTROLLED) return;
        if (getPosition() >= LIFT_MAX && v > 0) return;
        if (getPosition() <= 0 && v < 0) return;
        if (!ratchetEngaged) v /= 2;
        liftMode = LiftMode.DRIVER_CONTROLLED;
        resetCompletionActions();
        if (v <= 0 || !ratchetEngaged) internalSetVelocity(v);
    }

    public void setPower (double power) {
        disengageRatchet();
        internalSetVelocity(power);
    }

    private void internalSetVelocity(double v) {
        if (v != 0 && liftMode != LiftMode.HOLD_POSITION && !dumped && liftMode != LiftMode.FIND_BOTTOM) dumpMiddle();
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
        placer.openGate();
        liftRequested = true;
        liftTime = System.currentTimeMillis() + GATE_OPEN_WAIT_TIME;
    }

    private void executeLiftTop() {
        goToPosition(LIFT_SCORE);
        setDumpOnCompletion(DUMP_MIDDLE);
//        addCompletionAction(placerCloseAction);
        gateArmClosed = false;
        dumped = false;
        liftRequested = false;
    }

    public void liftBottom() {
        goToPosition(LIFT_DOWN);
        setDumpOnCompletion(DUMP_DOWN);
        addCompletionAction(findBottomAction);
        addCompletionAction(resetAction);
        dumped = false;
//        liftRequested = false;
    }

    public void dumpUp() {
        if (liftMode == LiftMode.RUN_TO_POSITION) setDumpOnCompletion(DUMP_UP);
        else if (getPosition() < LIFT_E_DUMP) {
            goToPosition(LIFT_E_DUMP);
            setDumpOnCompletion(DUMP_UP);
            actionsOnComplete.add(eDumpAction);
        } else {
            dump.setPosition(DUMP_UP);
            dumped = true;
        }
    }

    public void dumpMiddle() {
        dump.setPosition(DUMP_MIDDLE);
        dumped = false;
    }

    public void dumpDown() {
        dump.setPosition(DUMP_DOWN);
    }

    public void markerUp() {
    }

    public void markerDown() {
    }

    @Override
    public boolean isBusy() {
        return placer.isBusy() || (!asynch && Arrays.asList(LiftMode.RUN_TO_POSITION, LiftMode.LOWERING, LiftMode.FIND_LATCH).contains(liftMode));
    }

    public boolean isSensor() {
        return robot.getDigitalPort(0,0); //todo check this port
    }

    public boolean isAtLatch() {
        return !robot.getDigitalPort(0,3); //// TODO: check
    }

    public boolean isAtBottom() {
        return !robot.getDigitalPort(0, 5) && !robot.getDigitalPort(0, 7); //todo check
    }

    public boolean frameIsAtBottom() {
        return !robot.getDigitalPort(0, 5);
    }

    public void findLatch() {
        boolean below = getPosition() < (LIFT_FIND_LATCH_START_ABOVE + LIFT_FIND_LATCH_START_BELOW) / 2;
        goToPosition(below
                ? LIFT_FIND_LATCH_START_BELOW
                : LIFT_FIND_LATCH_START_ABOVE);
        addCompletionAction(findLatchAction);
        liftRequested = false;
        findLatchDirection = below ? 1 : -1;
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

    private double getFeedForward (MotionState state, double mass) {
        double ff = K_V * state.getV() + K_A * mass * (state.getA() - G);
        if (Math.abs(ff) > 1e-4) ff +=  Math.copySign(K_STATIC, ff);
        return ff;
    }

    private double getMass () {
        double mass = MASS_CARTRIGE;
        if (!frameIsAtBottom()) mass += MASS_FRAME;
        return mass;
    }

    private void resetCompletionActions () {
        actionsOnComplete.clear();
    }

    private void addCompletionAction (CompletionAction action) {
        if (!actionsOnComplete.contains(action)) actionsOnComplete.add(action);
    }

    private void executeCompletionActions () {
        for (CompletionAction action: actionsOnComplete) action.onCompletion();
        dumped = dumpPositionOnCompletion == DUMP_UP && actionsOnComplete.contains(dumpAction);
    }
}

