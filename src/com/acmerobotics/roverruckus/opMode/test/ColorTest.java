package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.util.ColorSpace;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="colorTest")
public class ColorTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        ColorSensor frontColor = hardwareMap.colorSensor.get("frontSensor");
        ColorSensor backColor = hardwareMap.colorSensor.get("backSensor");

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            int[] front = new int[3];
            int[] back = new int[3];
            ColorSpace.rgb2lab(frontColor.red(), frontColor.green(), frontColor.blue(), front);
            ColorSpace.rgb2lab(backColor.red(), backColor.green(), backColor.blue(), back);
            packet.put("front l", front[0]);
            packet.put("front a", front[1]);
            packet.put("front b", front[2]);

            packet.fieldOverlay().setFill("blue");
            packet.fieldOverlay().fillCircle(front[1], front[2], 5);
            packet.fieldOverlay().setFill(front[1] <= 0 ? "white" : "yellow");
            packet.fieldOverlay().fillRect(12, -24, 24, 24);

            packet.put("back l", back[0]);
            packet.put("back a", back[1]);
            packet.put("back b", back[2]);

            packet.fieldOverlay().setFill("red");
            packet.fieldOverlay().fillCircle(back[1], back[2], 5);
            packet.fieldOverlay().setFill(back[1] <= 0 ? "white" : "yellow");
            packet.fieldOverlay().fillRect(-12, -24, 24, 24);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
