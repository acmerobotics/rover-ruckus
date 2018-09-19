package com.acmerobotics.roverruckus.opmode.test;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="colorSensorTest")
public class ColorSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        ColorSensor colorSensor = hardwareMap.colorSensor.get("sensor");

        telemetry.addData("inited????", "probably");

        waitForStart();

        telemetry.clear();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("red", colorSensor.red());
            packet.put("blue", colorSensor.blue());
            packet.put("green", colorSensor.green());
            packet.put("alpha", colorSensor.alpha());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

    }
}
