package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.vision.SamplingVision;
import com.acmerobotics.roverruckus.vision.VisionCamera;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.CameraFrameGrabber;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "vision test")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        VisionCamera camera = new VisionCamera();
        SamplingVision samplingVision = new SamplingVision();
        samplingVision.enable();
        camera.addTracker(samplingVision);

        waitForStart();

        while (!isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("location", samplingVision.getLocation().toString());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

    }
}
