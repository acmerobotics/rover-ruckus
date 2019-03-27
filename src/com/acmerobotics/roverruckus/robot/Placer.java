package com.acmerobotics.roverruckus.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import android.graphics.Color;

@Config
public class Placer extends Subsystem {
    public static double armClose = .58;
    public static double armDivert = .35;
    public static double armOpen = .14;

    public static double gateOpen = .4;
    public static double gateClose = .1;

    public static double intakeOpen = .4;
    public static double intakeClose = .7;

    private ColorSensor frontColor, backColor;
    private DigitalChannel beamBreak;

    private Servo armServo, gateServo, intakeServo;

    private DcMotor intakeMotor;

    private boolean firstOut = false;
    private boolean secondOut = false;

    private boolean enabled = true;

    private boolean intaking = true;

    public enum Mineral {
        GOLD,
        SILVER,
        NONE,
        UNKNOWN
    }

    public Placer(HardwareMap map) {
        this.frontColor = map.colorSensor.get("frontSensor");
        this.backColor = map.colorSensor.get("backSensor");

        this.armServo = map.servo.get("diverter");
        this.gateServo = map.servo.get("spacer");
        this.intakeServo = map.servo.get("gate");

        this.intakeMotor = map.dcMotor.get("intakeMotor");


        armServo.setPosition(armOpen);
        gateServo.setPosition(gateOpen);

    }

    @Override
    public void update(TelemetryPacket packet) {

        if (!enabled) return;
    }

    public void releaseFirst() {
        firstOut = true;
        enabled = true;
    }

    public void releaseSecond() {
        secondOut = true;
        enabled = true;
    }

    public void reset() {
        firstOut = false;
        secondOut = false;
        gateServo.setPosition(gateOpen);
        armServo.setPosition(armOpen);
        intakeServo.setPosition(intakeOpen);
        enabled = false;
        intakeMotor.setPower(0);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void closeArm() {
        armServo.setPosition(armClose);
    }

    public void releaseGold() {
        armServo.setPosition(armOpen);
        if (firstOut) gateServo.setPosition(gateOpen);
        else gateServo.setPosition(gateClose);
        firstOut = true;
        intaking = false;

    }

    public void releaseSilver() {
        armServo.setPosition(armDivert);
        if (firstOut) gateServo.setPosition(gateOpen);
        else gateServo.setPosition(gateClose);
        firstOut = true;
        intaking = false;
    }

    public void closeIntake() {
        intakeServo.setPosition(intakeClose);
    }

    public void openIntake() {
        intakeServo.setPosition(intakeOpen);
    }

    public void openArm () {
        armServo.setPosition(armOpen);
    }

//    public void setIntakePower(double power) {
//        intakeMotor.setPower(power);
//    }
}
