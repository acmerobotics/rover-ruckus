package com.acmerobotics.roverruckus.opMode.auto;

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

import java.util.ArrayList;

@Config
@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {

    public static final String TAG = "autonomous";

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);
        SamplingVision.enable();
        RobotState state = new RobotState(hardwareMap.appContext);

        waitForStart();

        CameraFrameGrabber.getInstance().setOverlay(SamplingVision.processFrame(CameraFrameGrabber.getInstance().getFrame()));
        GoldLocation location = SamplingVision.getLocation();
        Log.i(TAG, location.toString());
        SamplingVision.disable();
        AutoPaths autoPaths = new AutoPaths(location, robot.config.getStartLocation(), robot.config.getSampleBoth());
        ArrayList<Trajectory> trajectories = autoPaths.paths();


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
                robot.lift.lower();
            }
        }

        robot.intake.retractRake();
        robot.waitForAllSubsystems();
        robot.intake.setIntakePower(1);
        robot.pause(1000);
        robot.intake.setIntakePower(0);

        state.setLiftOffset(robot.lift.getOffset());
        state.setRakeOffset(robot.intake.getOffset());

    }

}
