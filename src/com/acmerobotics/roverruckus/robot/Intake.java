package com.acmerobotics.roverruckus.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.hardware.CachingDcMotorEx;
import com.acmerobotics.roverruckus.hardware.CachingServo;
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
    public static double RAKE_VELOCITY = 1;
    public static double RAKE_UP = .95;
    public static double RAKE_DOWN = .5;
    public static double RAKE_STOW = .05;
    public static double DUMP_RESET = .95;
    public static double DUMP_DUMP = 0.05;

    private double rakePosition = 0;

    private CachingDcMotorEx rakeMotor, intakeMotor;

    private CachingServo rakeServo, dumpServo;

    public Intake(Robot robot, HardwareMap map) {

        rakeMotor = new CachingDcMotorEx(robot, map.get(DcMotorEx.class,"rakeMotor"), 1);
        rakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); //todo check direction
        robot.addMotor(rakeMotor);

        intakeMotor = new CachingDcMotorEx(robot, map.get(DcMotorEx.class, "intakeMotor"), 1);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addMotor(intakeMotor);

        rakeServo = new CachingServo(map.get(Servo.class, "rake"));
        robot.addMotor(rakeServo);

        dumpServo = new CachingServo(map.get(Servo.class, "dump"));
        robot.addMotor(dumpServo);
    }

    public void setArmPower(double power) {
        rakeMotor.setPower(power);
    }

    @Override
    public void update(TelemetryPacket packet) {
//        packet.put("intakeBusy", isBusy());
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    private void setRakePosition(double position) {
        rakePosition = position;
        rakeServo.setPosition(rakePosition);
    }

    public void rakeUp() {
        setRakePosition(RAKE_UP);
    }

    public void rakeStow() {
        setRakePosition(RAKE_STOW);
    }

    public void rakeDown() {
        setRakePosition(RAKE_DOWN);
    }

    long lastTime = 0;
    public void setRakeVelocity(double v) {
        if (lastTime == 0) {
            lastTime = System.currentTimeMillis();
        }
        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;
        lastTime = now;
        rakePosition += dt * RAKE_VELOCITY * v;
    }

    public void dumpDump() {
        dumpServo.setPosition(DUMP_DUMP);
    }

    public void resetDump() {
        dumpServo.setPosition(DUMP_RESET);
    }

}


