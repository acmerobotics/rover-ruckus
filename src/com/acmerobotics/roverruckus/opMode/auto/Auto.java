package com.acmerobotics.roverruckus.opMode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.trajectory.SplineTrajectory;
import com.acmerobotics.roverruckus.trajectory.Trajectory;
import com.acmerobotics.roverruckus.vision.GoldLocation;
import com.acmerobotics.roverruckus.vision.SamplingVision;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.CameraFrameGrabber;
import org.firstinspires.ftc.robotcontroller.internal.configuration.StartLocation;

import java.util.ArrayList;

@Config
@Autonomous(name="Auto")
public class Auto extends LinearOpMode {

    public static double RELEASE_X = 0;
    public static double RELEASE_Y = 2;

    public static final String TAG = "autonomous";

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);
        SamplingVision.enable();

        waitForStart();

        CameraFrameGrabber.getInstance().setOverlay(SamplingVision.processFrame(CameraFrameGrabber.getInstance().getFrame()));
        GoldLocation location = SamplingVision.getLocation();
        Log.i(TAG, location.toString());
        SamplingVision.disable();
        AutoPaths autoPaths = new AutoPaths(location, robot.config.getStartLocation());
        ArrayList<Trajectory> trajectories = autoPaths.paths();


        //lower
        if (robot.config.getLatched()) {
            Log.i(TAG, "lowering");
            robot.lift.lower();
            robot.waitForAllSubsystems();
        }
        robot.drive.setCurrentEstimatedPose(autoPaths.start().pos());

//        SplineTrajectory release = new TrajectoryBuilder(new Waypoint(robot.drive.getCurrentEstimatedPose(), -Math.PI / 2))
//                .to(new Waypoint(new Pose2d(RELEASE_X, RELEASE_Y, -Math.PI/12), -Math.PI/4))
//                .build().get(0);
//        Log.i(TAG, "path duration: " + release.duration());
//        robot.drive.followTrajectory(release);
//        robot.waitForAllSubsystems();
//        Log.i(TAG, "path complete");

//        autoPaths = new AutoPaths(location, robot.config.getStartLocation(), robot.drive.getCurrentEstimatedPose());


        for (int i = 0; i < trajectories.size(); i++) {
            robot.drive.followTrajectory(trajectories.get(i));
            robot.waitForAllSubsystems();
            if (i == 1 && robot.config.getStartLocation() == StartLocation.DEPOT) {
                robot.lift.markerUp();
                Log.e(TAG, "released marker");
            }
            if (i == 2 && robot.config.getStartLocation() == StartLocation.CRATER) {
                robot.lift.markerDown();
                Log.e(TAG, "released marker");
            }
        }

//        robot.intake.retractRake();
//        robot.waitForAllSubsystems();
//        robot.intake.setIntakePower(1);

        while (opModeIsActive());
//        robot.intake.setIntakePower(0);
    }

}
