package com.acmerobotics.roverruckus.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roverruckus.hardware.CachingDcMotorEx;
import com.acmerobotics.roverruckus.hardware.CachingServo;
import com.acmerobotics.roverruckus.util.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Emma Sheffo on 11/16/2018.
 */
@Config
public class Intake extends Subsystem{
    public static double RAKE_UP = .8;
    public static double RAKE_DOWN = .35;
    public static double WINCH_RADIUS = .5;
    public static double MAX_V = 10;
    public static double MAX_A = 10;
    public static double MAX_J = 10;
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double RAKE_RETRACT_DISTANCE = 2;

    private DcMotorEx rakeMotor, intakeMotor;

    private Servo rakeServo;

    private boolean driverControled = true;
    private double armPower = 0;
    private MotionProfile armProfile;
    private PIDController controller;
    private long startTime;
    private double offset = 0;

    public Intake(Robot robot, HardwareMap map) {

        rakeMotor =  map.get(DcMotorEx.class,"rakeMotor");
        rakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); //todo check direction
        rakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        setPosition(0);

        intakeMotor = map.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rakeServo = map.get(Servo.class, "rake");
        setPosition(0);
        controller = new PIDController(P, I, D);

    }

    public void setArmPower(double power) {
        armPower = power;
        driverControled = driverControled || Math.abs(power) > .1;
    }

    @Override
    public void update(TelemetryPacket packet) {
        packet.put("rakePosition", getPosition());
        if (driverControled) {
            rakeMotor.setPower(armPower);
        } else {
            long now = System.currentTimeMillis();
            double t = (now - startTime) / 1000.0;
            double targetV = 0;
            double targetX = armProfile.end().getX();
            if (t <= armProfile.duration()) {
                MotionState targetState = armProfile.get(t);
                targetV = targetState.getV();
                targetX = targetState.getX();
            }
            double error = getPosition() - targetX;
            double correction = controller.update(error);
            rakeMotor.setVelocity((targetV + correction) / WINCH_RADIUS);
        }
    }

    private void goToPosition(double position) {
        armProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getPosition(), 0, 0, 0),
                new MotionState(position, 0 ,0 ,0 ),
                MAX_V, MAX_A, MAX_J);
        driverControled = false;

        startTime = System.currentTimeMillis();

    }

    public double getPosition () {
       return rakeMotor.getCurrentPosition() / rakeMotor.getMotorType().getTicksPerRev() * (2 * Math.PI * WINCH_RADIUS)  + offset;
    }

    public void setPosition (double position) {
        offset = getPosition() - offset + position;
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
        }
        else {
            rakeDown();
        }
    }

    public void retractRake () {
        rakeUp();
        goToPosition(RAKE_RETRACT_DISTANCE);
    }


}


