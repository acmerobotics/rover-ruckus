package com.acmerobotics.roverruckus.robot;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roverruckus.util.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by Emma Sheffo on 11/16/2018.
 */
@Config
public class Intake extends Subsystem {
    public static final String TAG = "intake";
    public static double RAKE_UP = .7;
    public static double RAKE_DOWN = .24;
    public static double RAKE_MARKER_DEPLOY = .5;
    public static double GROUND_INTAKERER_IN = .7;
    public static double GROUND_INTAKERER_OUT = .1;
    public static double GROUND_INTAKERER_MIDDLE = .2;
    public static double WINCH_RADIUS = .71;
    public static double MAX_V = 30;
    public static double MAX_A = 30;
    public static double MAX_J = 15;
    public static double P = -10;
    public static double I = 0;
    public static double D = 0;
    public static double RAKE_RETRACT_DISTANCE = 12;
    public static double RAKE_MARKER_DISTANCE = 30;
    public static double EXPEL_DURATION = 1000;
    public static double RAKE_DELAY_TIME = 500;
    public static double DEBOUNCE_TIME = 500;
    public static double MAX_RAKE_EXTENSION = 40;

    private DcMotorEx rakeMotor, intakeMotor;

    private Servo rakeServo, groundIntakerer;

    private boolean driverControled = true;
    private boolean profileComplete = true;
    private double armPower = 0;
    private MotionProfile armProfile;
    private PIDController controller;
    private long startTime;
    private double offset = 0;
    private double targetX = 0;

    private AnalogInput beamBreak;
    private boolean intaking = false;
    private boolean needStop = false;
    private long stopTime;
    private double lastLow = 0;
    private double intakeTimout;

    private double rakeRetractBlockDistance = RAKE_RETRACT_DISTANCE + 2;

    private Robot robot;

    public Intake(Robot robot, HardwareMap map) {
        this.robot = robot;

        rakeMotor = robot.getMotor("rakeMotor");
        rakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); //todo check direction
        rakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = robot.getMotor("intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        beamBreak = robot.getAnalogInput(0, 1);

        rakeServo = robot.getServo("rake");
        groundIntakerer = robot.getServo("ground");
        setPosition(0);
        controller = new PIDController(P, I, D);
        driverControled = true;
        rakeUp();

    }

    public void setArmPower(double power) {
        armPower = power;
        driverControled = driverControled || Math.abs(power) > 0;
    }

    public void setRakeRetractBlockingDistance (double distance) {
        rakeRetractBlockDistance = RAKE_RETRACT_DISTANCE + distance;

    }

    @Override
    public void update(TelemetryPacket packet) {
        packet.put("rakePosition", getPosition());
        packet.put("beamBreak", beamBreak.getVoltage());
        Log.i(TAG, "position: " + getPosition());
        Log.i(TAG, "driver controlled: " + driverControled);
        Log.i(TAG, "complete" + profileComplete);
        if (driverControled) {
            rakeMotor.setPower(armPower);
            targetX = getPosition();

        } else {
            long now = System.currentTimeMillis();
            double t = (now - startTime) / 1000.0;
            Log.i(TAG, "t: " + t);
//            Log.i(TAG, "duration" + armProfile.duration());
            double targetV = 0;
            if (armProfile != null && t <= armProfile.duration()) {
                MotionState targetState = armProfile.get(t);
                targetV = targetState.getV();
                Log.i(TAG, "targetV: " + targetV);
                targetX = targetState.getX();
                Log.i(TAG, "targetX:" + targetX);
            } else {
                profileComplete = true;
            }
            double error = getPosition() - targetX;
            Log.i(TAG, "error" + error);
            packet.put("rakeError", error);
            double correction = controller.update(error);
            Log.i(TAG, "correction: " + correction);
            packet.put("command", targetV + correction);
            rakeMotor.setVelocity((targetV + correction) / WINCH_RADIUS, AngleUnit.RADIANS);
        }

        if (intaking) {
            if (beamBreak.getVoltage() > 500) lastLow = System.currentTimeMillis();
            if (beamBreak.getVoltage() < 500 && lastLow + DEBOUNCE_TIME <= System.currentTimeMillis() || System.currentTimeMillis() > intakeTimout) {
                intaking = false;
                intakeMotor.setPower(-1);
                needStop = true;
                stopTime = System.currentTimeMillis() + (long) EXPEL_DURATION;
            }
        }

        if (needStop) {
            if (stopTime <= System.currentTimeMillis()) {
                needStop = false;
                intakeMotor.setPower(0);
            }
        }
    }

    public void goToPosition(double position) {
        armProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getPosition(), 0, 0, 0),
                new MotionState(position, 0, 0, 0),
                MAX_V, MAX_A, MAX_J);
        driverControled = false;

        startTime = System.currentTimeMillis();
        profileComplete = false;

    }

    public double getPosition() {
        return rakeMotor.getCurrentPosition() / rakeMotor.getMotorType().getTicksPerRev() * (2 * Math.PI * WINCH_RADIUS) + offset;
    }

    public void setPosition(double position) {
        offset = -(getPosition() - offset + position);
    }


    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    private boolean rakeDown = false;

    public void rakeUp() {
        rakeServo.setPosition(RAKE_UP);
        rakeDown = false;
    }

    public void rakeDown() {
        rakeDown = true;
        rakeServo.setPosition(RAKE_DOWN);
    }

    public void toggleRake() {
        if (rakeDown) {
            rakeUp();
        } else {
            rakeDown();
        }
    }

    public void retractRake() {
        rakeUp();
        goToPosition(RAKE_RETRACT_DISTANCE);
        Log.e(TAG, "retracting");
    }

    public void groundIntakererIn () {
        groundIntakerer.setPosition(GROUND_INTAKERER_IN);
    }

    public void groundIntakererOut () {
        groundIntakerer.setPosition(GROUND_INTAKERER_OUT);
    }

    public void groundIntakererMiddle () {groundIntakerer.setPosition(GROUND_INTAKERER_MIDDLE);}

    public void deployMarker () {
        rakeServo.setPosition(RAKE_MARKER_DEPLOY);
        robot.pause(RAKE_DELAY_TIME);
        rakeServo.setPosition(RAKE_UP);
    }

    public void extendToDeploy () {
        goToPosition(RAKE_MARKER_DISTANCE);
    }

    @Override
    public boolean isBusy() {
        return (!profileComplete && getPosition() > rakeRetractBlockDistance) || intaking;
    }

    public double getOffset() {
        return offset;
    }

    public void setOffset(double offset) {
        this.offset = 0;
    }

    public void intake () {
        groundIntakererIn();
        intake(30);
    }

    public void intake (double timeout) {
        intaking = true;
        intakeMotor.setPower(1);
        intakeTimout = System.currentTimeMillis() + timeout * 1000;
    }

}


