package com.acmerobotics.roverruckus.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Placer extends Subsystem {
    public static double armClose = .58;
    public static double armDivert = .35;
    public static double armOpen = .08;

    public static double gateOpen = .4;
    public static double gateClose = .1;

    public static double intakeOpen = .4;
    public static double intakeClose = .7;

    public static double colorThreshold = 1.5;
    public static double distanceThreshold = 6;

    public static int delay = 500;

    private ColorSensor frontColor, backColor;
    private DistanceSensor frontDistance, backDistance;
    private AnalogInput beamBreak;

    private Servo armServo, gateServo, intakeServo;

    private boolean firstIn = false;
    private boolean secondIn = false;
    private boolean firstOut = false;
    private boolean secondOut = false;

    private long waitTime = 0;
    private boolean enabled = true;

    private boolean intaking = true;

    public enum Mineral {
        GOLD,
        SILVER,
        NONE,
        UNKNOWN
    }

    public Placer(Robot robot, HardwareMap map) {
        this.frontColor = map.colorSensor.get("frontSensor");
        this.backColor = map.colorSensor.get("backSensor");
        this.frontDistance = map.get(DistanceSensor.class, "frontSensor");
        this.backDistance = map.get(DistanceSensor.class, "backSensor");

        this.armServo = map.servo.get("diverter");
        this.gateServo = map.servo.get("spacer");
        this.intakeServo = map.servo.get("gate");
        beamBreak = robot.getAnalogInput(0, 1);

        armServo.setPosition(armOpen);
        gateServo.setPosition(gateOpen);

    }

    @Override
    public void update(TelemetryPacket packet) {
        packet.put("enabled", enabled);
        packet.put("beamBreak", beamBreak.getVoltage());
        if (!enabled) return;


    }

    private Mineral getMineral(ColorSensor sensor, DistanceSensor distanceSensor) {
        if (distanceSensor.getDistance(DistanceUnit.CM) > distanceThreshold) return Mineral.NONE;
        double ratio = ((double) sensor.red()) / ((double) sensor.blue() + .0001);
        if (ratio < colorThreshold) return Mineral.SILVER;
        return Mineral.GOLD;
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
        firstIn = false;
        secondIn = false;
        firstOut = false;
        secondOut = false;
        gateServo.setPosition(gateOpen);
        armServo.setPosition(armOpen);
//        intakeServo.setPosition(intakeOpen);
        enabled = false;
//        intakeMotor.setPower(0);

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

    public void closeGate () {
        gateServo.setPosition(gateClose);
    }

    public void openGate () {
        gateServo.setPosition(gateOpen);
    }

//    public void setIntakePower(double power) {
//        intakeMotor.setPower(power);
//    }
}
