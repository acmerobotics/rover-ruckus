package com.acmerobotics.roverruckus.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Placer extends Subsystem {
    public static double armClose = .58;
    public static double armDivert = .3;
    public static double armOpen = .14;

    public static double gateOpen = .4;
    public static double gateClose = .1;

    public static double intakeOpen = 0;
    public static double intakeClose = 0;

    public static double colorThreshold = 1.5;
    public static double distanceThreshold = 6;

    public static int delay = 500;

    private ColorSensor frontColor, backColor;
    private DistanceSensor frontDistance, backDistance;

    private Servo armServo, gateServo, intakeServo;

//    private DcMotor intakeMotor;

    private boolean firstIn = false;
    private boolean secondIn = false;
    private boolean firstOut = false;
    private boolean secondOut = false;

    private long waitTime = 0;
    private boolean enabled = true;

    public enum Mineral {
        GOLD,
        SILVER,
        NONE,
        UNKNOWN
    }

    public Placer (HardwareMap map) {
        this.frontColor = map.colorSensor.get("frontSensor");
        this.backColor = map.colorSensor.get("backSensor");
        this.frontDistance = map.get(DistanceSensor.class, "frontSensor");
        this.backDistance = map.get(DistanceSensor.class, "backSensor");

        this.armServo = map.servo.get("diverter");
        this.gateServo = map.servo.get("spacer");
        this.intakeServo = map.servo.get("gate");

//        this.intakeMotor = map.dcMotor.get("intakeMotor");


        armServo.setPosition(armClose);
        gateServo.setPosition(gateOpen);

    }

    @Override
    public void update(TelemetryPacket packet) {
        packet.put("enabled", enabled);
        if (!enabled) return;
        packet.put("front ratio", (double) frontColor.red() / ((double) frontColor.blue()) + .0001);
        packet.put("back ratio", (double) backColor.red() / ((double) backColor.blue()) + .0001);

        packet.put("front dist", frontDistance.getDistance(DistanceUnit.CM));
        packet.put("back dist", backDistance.getDistance(DistanceUnit.CM));

        packet.put("first in", firstIn);
        packet.put("first out", firstOut);
        packet.put("second in", secondIn);
        packet.put("second out", secondOut);


        Mineral back = getMineral(backColor, backDistance);
        Mineral front = getMineral(frontColor, frontDistance);

        packet.put("frontCurrent", front.toString());
        packet.put("backCurrent", back.toString());

        if (System.currentTimeMillis() < waitTime) return;

        if (back != Mineral.NONE && !firstIn) {
           gateServo.setPosition(gateClose);
           waitTime = System.currentTimeMillis() + delay;
           firstIn = true;
        } else if (!secondIn && back != Mineral.NONE && front != Mineral.NONE) {
            intakeServo.setPosition(intakeClose);
            secondIn = true;
            enabled = false;
        }

        if (secondOut) {
            armServo.setPosition(back == Mineral.SILVER ? armDivert : armOpen);
            gateServo.setPosition(gateOpen);
            enabled = false;
        }
        else if (firstOut) {
            armServo.setPosition(front == Mineral.SILVER ? armDivert : armOpen);
            if (front == back) gateServo.setPosition(gateOpen);
            enabled = false;
        }




    }

    private Mineral getMineral (ColorSensor sensor, DistanceSensor distanceSensor) {
        if (distanceSensor.getDistance(DistanceUnit.CM) > distanceThreshold) return Mineral.NONE;
        double ratio = ((double) sensor.red()) / ((double) sensor.blue() + .0001);
        if (ratio < colorThreshold) return Mineral.SILVER;
        return Mineral.GOLD;
    }

    public void releaseFirst () {
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
        intakeServo.setPosition(intakeOpen);
        enabled = false;
//        intakeMotor.setPower(0);

    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void closeArm() {
        armServo.setPosition(armClose);
    }

//    public void setIntakePower(double power) {
//        intakeMotor.setPower(power);
//    }
}
