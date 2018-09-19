package com.acmerobotics.roverruckus.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="sorterTest")
public class SorterTest extends LinearOpMode{

    public static double threshold = 2;

    public static double center = .5;
    public static double silver = 0;
    public static double gold = 1;

    @Override
    public void runOpMode() {
//        Servo servo = hardwareMap.get(Servo.class, "servo");
        ColorSensor sensor = hardwareMap.get(ColorSensor.class, "sensor");

        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();

        while(!isStopRequested()) {
            double red = sensor.red();
            double green = sensor.green();
            double blue = sensor.blue();

            double ratio = red / blue + .0001;

            String color = ratio < threshold? "silver": "gold";

            packet.put("red", red);
            packet.put("green", green);
            packet.put("blue", blue);

            packet.put("ratio", ratio);
            packet.put("color", color);

//            if (gamepad1.a) {
//                servo.setPosition(silverDistance < goldDistance ? silver : gold);
//            }
//
//            if (gamepad1.b) {
//                servo.setPosition(center);
//            }

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

    }

}
