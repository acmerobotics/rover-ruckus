package com.acmerobotics.roverruckus.opMode.auto;

import android.media.MediaPlayer;
import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roverruckus.robot.Intake;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.trajectory.TrajectoryBuilder;
import com.acmerobotics.roverruckus.trajectory.Waypoint;
import com.acmerobotics.roverruckus.vision.GoldLocation;
import com.acmerobotics.roverruckus.vision.SamplingVision;
import com.acmerobotics.roverruckus.vision.VisionCamera;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.configuration.StartLocation;
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

    protected void setRakePosition(Vector2d rakePosition) {
        Pose2d robotPosition = robot.drive.getCurrentEstimatedPose();
        double distance = rakePosition.distanceTo(robotPosition.pos()) - 6;
        double angle = rakePosition.minus(robotPosition.pos()).angle();
        if (distance > Intake.MAX_RAKE_EXTENSION) {
            double moveDistance = distance - Intake.MAX_RAKE_EXTENSION;
            distance -= moveDistance;
            Pose2d moveDestination = new Pose2d(new Vector2d(1, 0).rotated(angle).times(moveDistance), angle);
            robot.drive.followTrajectory(new TrajectoryBuilder(new Waypoint(robotPosition, angle)).to(new Waypoint(moveDestination, angle)).build().get(0));
        } else {
            robot.drive.followTrajectory(new TrajectoryBuilder(new Waypoint(robotPosition, angle)).turnTo(angle).build().get(0));
        }
        robot.intake.goToPosition(distance);
        robot.waitForAllSubsystems();
    }

    public AutoAction lowerLift = () -> {
        robot.lift.setAsynch(true);
        robot.lift.lower();
    };

    public AutoAction extendRake = () ->
        setRakePosition(robot.config.getStartLocation() == StartLocation.CRATER
                ? AutoPaths.RAKE_POSITION_CRATER
                : AutoPaths.RAKE_POSITION_DEPOT);

    public AutoAction deployMarker = () -> robot.lift.releaseMarker();

    public AutoAction retractRake = () -> robot.intake.retractRake();

    public AutoAction startIntake = () -> robot.intake.setIntakePower(1);

    public AutoAction reverseIntake = () -> robot.intake.setIntakePower(-1);

    public AutoAction stopIntake = () -> {
        robot.intake.setIntakePower(0);
        robot.intake.groundIntakererIn();
    };

    public AutoAction groundIntake = () -> {
        robot.intake.setIntakePower(1);
        robot.intake.groundIntakererOut();
    };

    public AutoAction blockingIntake = () -> robot.intake.intake();

    public AutoAction raiseLift = () -> {
        robot.lift.liftTop();
        robot.lift.setAsynch(false);
    };

    public AutoAction score = () -> {
        robot.lift.dumpUp();
        robot.pause(1000);
        robot.lift.placer.release();
        robot.waitForAllSubsystems();
        robot.pause(1000);
        robot.lift.lower();
        robot.pause(500);
        robot.lift.setAsynch(true);
    };

    public Robot getRobot() {
        return robot;
    }
}
