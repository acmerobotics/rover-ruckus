package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.roverruckus.vision.SamplingVision;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.CameraFrameGrabber;

@Autonomous(name="vision test")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        SamplingVision vision = new SamplingVision();

        waitForStart();

        while (!isStopRequested()) {
            CameraFrameGrabber.getInstance().setOverlay(vision.processFrame(CameraFrameGrabber.getInstance().getFrame()));
        }
    }
}
