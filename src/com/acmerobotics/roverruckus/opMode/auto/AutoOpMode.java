package com.acmerobotics.roverruckus.opMode.auto;

import android.media.MediaPlayer;
import android.util.Log;

import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.vision.GoldLocation;
import com.acmerobotics.roverruckus.vision.SamplingVision;
import com.acmerobotics.roverruckus.vision.VisionCamera;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;


public abstract class AutoOpMode extends LinearOpMode {

    public static final String TAG = "autonomous";

    protected Robot robot;
    protected VisionCamera camera;

    @Override
    public void runOpMode() {

        robot = new Robot(this, hardwareMap);
        camera = new VisionCamera();
        SamplingVision samplingVision = new SamplingVision();
        camera.addTracker(samplingVision);
        samplingVision.enable();

        MediaPlayer media = null;
        try {
            if (robot.config.getPlayMusic())
                media = MediaPlayer.create(hardwareMap.appContext, R.raw.tokyo_drift);
        } catch (Exception e) {
            Log.e(TAG, "error playing media: " + e.getMessage());
        }

        waitForStart();

        GoldLocation location = samplingVision.getLocation();
        samplingVision.disable();

        Log.i(TAG, location.toString());

        if (media != null) media.start();

        robot.pause(robot.config.getDelay() * 1000);

        //lower
        if (robot.config.getLatched()) {
            robot.lift.lower();
            robot.waitForAllSubsystems();
        }

        run();

        if (media != null) {
            media.stop();
            media.release();
        }
    }

    protected abstract void run ();

    public AutoAction lowerLift = () -> {
        robot.lift.setAsynch(true);
        robot.lift.lower();
    };

    public AutoAction deployMarker = () -> robot.lift.releaseMarker();

    public AutoAction retractRake = () -> robot.intake.retractRake();




}
