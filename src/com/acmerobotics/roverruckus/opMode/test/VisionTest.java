package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.vision.SamplingVision;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.CameraFrameGrabber;

@Autonomous(name="vision test")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        waitForStart();

        while (!isStopRequested()) {
            CameraFrameGrabber.getInstance().setOverlay(SamplingVision.processFrame(CameraFrameGrabber.getInstance().getFrame()));
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x", SamplingVision.getX());
            packet.put("y", SamplingVision.getY());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
