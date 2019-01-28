package com.acmerobotics.roverruckus.opMode.test;


import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.vision.SamplingVision;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.CameraFrameGrabber;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Autonomous(name="vision test")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        waitForStart();

        while (!isStopRequested()) {
            Mat frame = CameraFrameGrabber.getInstance().getFrame();
            frame = SamplingVision.processFrame(frame);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("frame", frame.channels());
            packet.put("0,0", frame.get(0,0));
            packet.put("yup", frame);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            Imgproc.circle(frame, new Point(10, 10), 10, new Scalar(1, 0, 0), 5);
            CameraFrameGrabber.getInstance().setOverlay(frame);
        }
    }
}
