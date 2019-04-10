package com.acmerobotics.roverruckus.opMode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roverruckus.trajectory.Trajectory;
import com.acmerobotics.roverruckus.trajectory.TrajectoryBuilder;
import com.acmerobotics.roverruckus.trajectory.Waypoint;
import com.acmerobotics.roverruckus.util.SuperArrayList;
import com.acmerobotics.roverruckus.vision.GoldLocation;

import org.firstinspires.ftc.robotcontroller.internal.configuration.StartLocation;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcontroller.internal.configuration.StartLocation.CRATER;

@Config
public class AutoPaths {

    private static double PI = Math.PI;
    private static final double SAMPLE_DIST = 7;
    public static double START_DIST = 19 / Math.sqrt(2);
    public static double RELEASE_X = 1;
    public static double RELEASE_Y = 2;
    public static double RELEASE_THETA = -PI / 12;

    public static final Vector2d RAKE_POSITION_CRATER = new Vector2d(60, -48);
    public static final Vector2d RAKE_POSITION_DEPOT = new Vector2d(60, 60);

    private static final Pose2d RELEASE = new Pose2d(RELEASE_X, RELEASE_Y, RELEASE_THETA);

    private static final Waypoint START_CRATER = new Waypoint(new Pose2d(START_DIST, -START_DIST, -PI / 4), PI / 4);
    private static final Waypoint START_DEPOT = new Waypoint(new Pose2d(START_DIST, START_DIST, PI / 4), 3 * PI / 4);

    private static final Waypoint RELEASE_CRATER = new Waypoint(new Pose2d(START_CRATER.pos().pos()
            .plus(RELEASE.pos().rotated(START_CRATER.pos().getHeading())),
            -PI / 3),
            0, -PI / 4);

    private static final Waypoint RELEASE_DEPOT = new Waypoint(new Pose2d(START_DEPOT.pos().pos()
            .plus(RELEASE.pos().rotated(START_DEPOT.pos().getHeading())),
            PI / 6),
            PI / 2, PI / 4);

    private static final Waypoint MARKER_CRATER = new Waypoint(new Pose2d(60, 24, PI/2), PI / 2, -PI / 2);
    private static final Waypoint MARKER_DEPOT = new Waypoint(new Pose2d(48, 60, -PI / 2), PI / 2, PI);

    private static final Waypoint PARK_CRATER = new Waypoint(new Pose2d(60, -17, -PI / 2),-PI/2);
    private static final Waypoint PARK_DEPOT = new Waypoint(new Pose2d(-8, 60, -PI), -PI);

    private static final Waypoint SAMPLE_LEFT_CRATER = new Waypoint(new Pose2d(36 + SAMPLE_DIST, -(12 + SAMPLE_DIST), -PI / 4), -PI / 4, PI / 4);
    private static final Waypoint SAMPLE_CENTER_CRATER = new Waypoint(new Pose2d(24 + SAMPLE_DIST, -(24 + SAMPLE_DIST), -PI / 4), -PI / 4, 3 * PI / 4);
    private static final Waypoint SAMPLE_RIGHT_CRATER = new Waypoint(new Pose2d(12 + SAMPLE_DIST, -(36 + SAMPLE_DIST), -PI / 4), -PI / 4, 3 * PI / 4);

    private static final Waypoint SAMPLE_LEFT_DEPOT = new Waypoint(new Pose2d(24, 48, PI / 4), PI / 4);
    private static final Waypoint SAMPLE_CENTER_DEPOT = new Waypoint(new Pose2d(36, 36, PI / 4), PI / 3);
    private static final Waypoint SAMPLE_RIGHT_DEPOT = new Waypoint(new Pose2d(48, 24, PI / 4), PI / 4);

    private static final Waypoint CLEAR_ONE_CRATER = new Waypoint(new Pose2d(36, -16, -PI / 2), PI / 5);
    private static final Waypoint CLEAR_TWO_CRATER = new Waypoint(new Pose2d(52, 0, -PI/2), PI/4);

    private Waypoint lastPosition;

    private GoldLocation location;
    private StartLocation start;
    private AutoOpMode opMode;

    public AutoPaths(AutoOpMode opMode, GoldLocation location, StartLocation start) {
        this.location = location;
        this.start = start;
        this.lastPosition = start();
        this.opMode = opMode;
    }

    public Waypoint start() {
        return start == CRATER ? START_CRATER : START_DEPOT;
    }

    public Waypoint release() {
        return start == CRATER ? RELEASE_CRATER : RELEASE_DEPOT;
    }

    public Waypoint marker() {
        return start == CRATER ? MARKER_CRATER : MARKER_DEPOT;
    }

    public Waypoint park() {
        return start == CRATER ? PARK_CRATER : PARK_DEPOT;
    }

    public Waypoint sample () {
        switch (location) {
            case RIGHT: return start == CRATER ? SAMPLE_RIGHT_CRATER : SAMPLE_RIGHT_DEPOT;
            case CENTER: return start == CRATER ? SAMPLE_CENTER_CRATER : SAMPLE_CENTER_DEPOT;
            default: return start == CRATER ? SAMPLE_LEFT_CRATER : SAMPLE_LEFT_DEPOT;
        }
    }

    public SuperArrayList<Trajectory> startToRelease () {
        TrajectoryBuilder builder = getBuilder();
        lastPosition = release();
        return builder
                .to(release())
                .addActionOnCompletion(opMode.lowerLift)
                .build();
    }

    public SuperArrayList<Trajectory> toSample () {
        TrajectoryBuilder builder = getBuilder();
        lastPosition = sample();
        return builder.to(sample()).addActionOnStart(opMode.groundIntake).build();
    }

//    public SuperArrayList<Trajectory> toMarker () {
//
//    }

    private TrajectoryBuilder getBuilder () {
        return new TrajectoryBuilder(new Waypoint(opMode.getRobot().drive.getCurrentEstimatedPose(), lastPosition.getExit().getHeading()));
    }

}
