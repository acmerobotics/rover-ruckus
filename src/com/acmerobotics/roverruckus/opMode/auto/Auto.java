package com.acmerobotics.roverruckus.opMode.auto;

import android.media.MediaPlayer;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.robot.RobotState;
import com.acmerobotics.roverruckus.trajectory.Trajectory;
import com.acmerobotics.roverruckus.vision.GoldLocation;
import com.acmerobotics.roverruckus.vision.SamplingVision;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.CameraFrameGrabber;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {

    public static final String TAG = "autonomous";

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this, hardwareMap);

        MediaPlayer media = null;
        try {
            if (robot.config.getPlayMusic())
                media = MediaPlayer.create(hardwareMap.appContext, R.raw.tokyo_drift);
        } catch (Exception e) {
            Log.e(TAG, "error playing media: " + e.getMessage());
        }

        SamplingVision.enable();
        RobotState state = new RobotState(hardwareMap.appContext);

        waitForStart();

        CameraFrameGrabber.getInstance().setOverlay(SamplingVision.processFrame(CameraFrameGrabber.getInstance().getFrame()));
        GoldLocation location = SamplingVision.getLocation();
        Log.i(TAG, location.toString());
        SamplingVision.disable();
        AutoPaths autoPaths = new AutoPaths(location, robot.config.getStartLocation(), robot.config.getSampleBoth());
        ArrayList<Trajectory> trajectories = autoPaths.paths();

        if (media != null) media.start();

        //lower
        if (robot.config.getLatched()) {
            Log.i(TAG, "lowering");
            robot.lift.lower();
            robot.waitForAllSubsystems();
        }
        robot.drive.setCurrentEstimatedPose(autoPaths.start().pos());

        for (Trajectory trajectory : trajectories) {
            robot.drive.followTrajectory(trajectory);
            robot.waitForAllSubsystems();

            if (trajectory.containsFlag(AutoFlag.RELEASE_MARKER)) {
                robot.lift.releaseMarker();
            }

            if (trajectory.containsFlag(AutoFlag.LOWER_LIFT)) {
                robot.lift.setAsynch(true);
                robot.lift.goToPosition(0);
            }
        }

        robot.intake.retractRake();
        robot.waitForAllSubsystems();
        robot.intake.setIntakePower(1);
        robot.pause(1000);
        robot.intake.setIntakePower(0);


        if (media != null) {
            media.stop();
            media.release();
        }


    }

}
