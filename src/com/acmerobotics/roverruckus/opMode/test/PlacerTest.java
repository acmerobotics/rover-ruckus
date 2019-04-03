package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.robot.Placer;
import com.acmerobotics.roverruckus.util.ColorSpace;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "placerTest")
public class PlacerTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        ColorSensor front = hardwareMap.colorSensor.get("frontSensor");

        waitForStart();

        while (!isStopRequested()) {

            int r = front.red();
            int g = front.green();
            int b = front.blue();

            int[] lab = new int[3];

            ColorSpace.rgb2lab(r, g, b, lab);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("l", lab[0]);
            packet.put("a", lab[1]);
            packet.put("b", lab[2]);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }
    }

}
