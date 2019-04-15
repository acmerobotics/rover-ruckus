package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.vision.SamplingVision;
import com.acmerobotics.roverruckus.vision.VisionCamera;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "vision test")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        VisionCamera camera = new VisionCamera();
        SamplingVision samplingVision = new SamplingVision(this);
        samplingVision.enable();
        camera.addTracker(samplingVision);

        waitForStart();

        while (!isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("location", samplingVision.getLocation().toString());
            packet.put("x", samplingVision.getX());
            packet.put("y", samplingVision.getY());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        samplingVision.disable();

    }
}
