package com.acmerobotics.roverruckus.opMode.auto;

import android.media.MediaPlayer;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public abstract class AutoOpMode extends LinearOpMode {

    public static double MIN_INTAKE_TIME = 5;
    public static double MIN_SCORE_TIME = 10;

    public static final String TAG = "autonomous";

    protected Robot robot;
    protected VisionCamera camera;
    protected GoldLocation goldLocation;
    protected StartLocation startLocation;
    private double startTime;

    @Override
    public void runOpMode() {

        robot = new Robot(this, hardwareMap);
        camera = new VisionCamera();
        SamplingVision samplingVision = new SamplingVision(this);
        camera.addTracker(samplingVision);
        samplingVision.enable();

        MediaPlayer media = null;
        try {
            if (robot.config.getPlayMusic())
                media = MediaPlayer.create(hardwareMap.appContext, R.raw.tokyo_drift);
        } catch (Exception e) {
            Log.e(TAG, "error playing media: " + e.getMessage());
        }

        robot.lift.setCalibrated();

        waitForStart();
        startTime = System.currentTimeMillis();

        goldLocation = samplingVision.getLocation();
        samplingVision.disable();
        startLocation = robot.config.getStartLocation();

        Log.i(TAG, goldLocation.toString());

        if (media != null) media.start();

        robot.pause(robot.config.getDelay() * 1000);

        //lower
        robot.lift.setPosition(0);
        robot.intake.setPosition(0);
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
            Log.i(TAG, "moving forward: " + (distance - Intake.MAX_RAKE_EXTENSION));
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
        robot.lift.liftBottom();
    };

    public AutoAction extendRake = () ->
        setRakePosition(robot.config.getStartLocation() == StartLocation.CRATER
                ? AutoPaths.RAKE_POSITION_CRATER
                : AutoPaths.RAKE_POSITION_DEPOT);

    public AutoAction deployMarker = () -> robot.intake.deployMarker();

    public AutoAction retractRake = () -> {
        robot.intake.retractRake();
    };

    public AutoAction startIntake = () -> robot.intake.setIntakePower(.9);

    public AutoAction reverseIntake = () -> robot.intake.setIntakePower(-1);

    public AutoAction stopIntake = () -> {
        robot.intake.setIntakePower(0);
        robot.intake.groundIntakererIn();
    };

    public AutoAction groundIntake = () -> {
        robot.intake.setIntakePower(.8);
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

    public double runTime () {
        return (System.currentTimeMillis() - startTime) / 1000.0;
    }

    public double remainingTime () {
        return 30 - runTime();
    }
}
