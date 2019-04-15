package com.acmerobotics.roverruckus.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.util.ColorSpace;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
    public static double armDivert = .3;
    public static double armOpen = .08;

    public static double gateOpen = .51;
    public static double gateClose = .15;

    public static double intakeOpen = .8;
    public static double intakeClose = .27;

    public static double RELEASE_DELAY = 1000;

    private ColorSensor frontColor, backColor;

    private Servo armServo, gateServo, intakeServo;

    private boolean firstOut = false;

    private boolean release = false;
    private double second = 0;

    public enum Mineral {
        GOLD,
        SILVER,
        NONE,
        UNKNOWN
    }

    public Placer(Robot robot, HardwareMap map) {
        this.frontColor = map.colorSensor.get("frontSensor");
        this.backColor = map.colorSensor.get("backSensor");

        this.armServo = map.servo.get("diverter");
        this.gateServo = map.servo.get("spacer");
        this.intakeServo = map.servo.get("gate");

        armServo.setPosition(armOpen);
        gateServo.setPosition(gateOpen);

    }

    @Override
    public void update(TelemetryPacket packet) {
        if (release && System.currentTimeMillis() >= second) {
            setArmPosition(getColor(backColor));
            gateServo.setPosition(gateOpen);
            release = false;
        }
    }

    public void release() {
        release = false;
        setArmPosition(getColor(frontColor));
        if (getColor(frontColor) == getColor(backColor)) gateServo.setPosition(gateOpen);
        else {
            gateServo.setPosition(gateClose);
            second = System.currentTimeMillis() + RELEASE_DELAY;
            release = true;
        }
    }


    public void reset() {
        firstOut = false;
        closeGate();
        armServo.setPosition(armOpen);
        intakeServo.setPosition(intakeOpen);
    }


    public void closeArm() {
        armServo.setPosition(armClose);
    }

    public void setArmPosition (Mineral mineral) {
        armServo.setPosition(mineral == Mineral.SILVER ? armDivert : armOpen);
    }

    public Mineral getColor (ColorSensor sensor) {
        int[] lab = new int[3];
        ColorSpace.rgb2lab(sensor.red(), sensor.green(), sensor.blue(), lab);
        return lab[1] <= 0 ? Mineral.SILVER : Mineral.GOLD;
    }

    public void releaseGold() {
        armServo.setPosition(armOpen);
        if (firstOut) gateServo.setPosition(gateOpen);
        else gateServo.setPosition(gateClose);
        firstOut = true;

    }

    public void releaseSilver() {
        armServo.setPosition(armDivert);
        if (firstOut) gateServo.setPosition(gateOpen);
        else gateServo.setPosition(gateClose);
        firstOut = true;
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

    public void closeGate () {
        gateServo.setPosition(gateClose);
    }

    public void openGate () {
        gateServo.setPosition(gateOpen);
    }

    @Override
    public boolean isBusy() {
        return release;
    }
}
