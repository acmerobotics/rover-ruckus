package com.acmerobotics.roverruckus.opMode.test;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.vision.SamplingVision;
import com.acmerobotics.roverruckus.vision.Tracker;
import com.acmerobotics.roverruckus.vision.VisionCamera;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Mat;

@Autonomous(name = "vision test")
public class VisionTest extends LinearOpMode implements Tracker {
    @Override
    public void runOpMode() {

        VisionCamera camera = new VisionCamera();
        camera.addTracker(this);

        waitForStart();

        while (!isStopRequested()) { /* nothing */ }

    }

    @Override
    public void processFrame(Mat frame) {
        Log.i("vision", "width: " + frame.width() + " height: " + frame.height());

    }
}
