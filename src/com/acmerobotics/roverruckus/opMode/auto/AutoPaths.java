package com.acmerobotics.roverruckus.opMode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roverruckus.trajectory.SplineTrajectory;
import com.acmerobotics.roverruckus.trajectory.Trajectory;
import com.acmerobotics.roverruckus.trajectory.TrajectoryBuilder;
import com.acmerobotics.roverruckus.trajectory.Waypoint;
import com.acmerobotics.roverruckus.vision.GoldLocation;

import org.firstinspires.ftc.robotcontroller.internal.configuration.StartLocation;

import java.util.ArrayList;

@Config
public class AutoPaths {

    private static double PI = Math.PI;
    private static final double SAMPLE_DIST = 7;
    public static double START_DIST = 19 / Math.sqrt(2);
    public static double RELEASE_X = 1;
    public static double RELEASE_Y = 2;
    public static double RELEASE_THETA = -PI/12;

    private static final Pose2d RELEASE = new Pose2d(RELEASE_X, RELEASE_Y, RELEASE_THETA);

    private static final Waypoint START_CRATER = new Waypoint(new Pose2d(START_DIST, -START_DIST, -PI/4), PI/4);
    private static final Waypoint START_DEPOT = new Waypoint(new Pose2d(START_DIST, START_DIST, PI/4), 3*PI/4);

    private static final Waypoint RELEASE_CRATER = new Waypoint(new Pose2d(START_CRATER.pos().pos()
            .plus(RELEASE.pos().rotated(START_CRATER.pos().getHeading())),
            -PI/3),
            0, -PI/4);

    private static final Waypoint RELEASE_DEPOT= new Waypoint(new Pose2d(START_DEPOT.pos().pos()
            .plus(RELEASE.pos().rotated(START_DEPOT.pos().getHeading())),
            PI/6),
            PI/2, PI/4);

    private static final Waypoint MARKER_CRATER = new Waypoint(new Pose2d(60, 48, 0), PI/2, -PI/2);
    private static final Waypoint MARKER_DEPOT = new Waypoint(new Pose2d(48, 60, -PI/2), PI/2, PI);
    private static final Waypoint MARKER_DEPOT_LEFT = new Waypoint(new Pose2d(48, 60, -PI/2), 0, PI);

    private static final Waypoint PARK_CRATER = new Waypoint(new Pose2d(60, -12, -PI/2), -PI/2);
    private static final Waypoint PARK_DEPOT = new Waypoint(new Pose2d(-12, 60, -PI), -PI);

    private static final Waypoint SAMPLE_LEFT_CRATER = new Waypoint(new Pose2d(36 + SAMPLE_DIST, -(12 + SAMPLE_DIST), PI/4), -PI/4, PI/4);
    private static final Waypoint SAMPLE_CENTER_CRATER = new Waypoint(new Pose2d(24+ SAMPLE_DIST, -(24 + SAMPLE_DIST), PI/4), -PI/4, 3*PI/4);
    private static final Waypoint SAMPLE_RIGHT_CRATER = new Waypoint(new Pose2d(12+ SAMPLE_DIST, -(36 + SAMPLE_DIST), PI/4), -PI/4, 3*PI/4);

    private static final Waypoint SAMPLE_LEFT_DEPOT = new Waypoint(new Pose2d(24, 48, -PI/4), PI/4);
    private static final Waypoint SAMPLE_CENTER_DEPOT = new Waypoint(new Pose2d(36, 36, -PI/4), PI/3);
    private static final Waypoint SAMPLE_RIGHT_DEPOT = new Waypoint(new Pose2d(48, 24, -PI/4), PI/4);

    private static final Waypoint CLEAR_ONE_CRATER = new Waypoint(new Pose2d(36, -12, PI/4), PI/4);
    private static final Waypoint CLEAR_TWO_CRATER = new Waypoint(new Pose2d(60, 12, 0), PI/2);

    private GoldLocation location;
    private StartLocation start;
    private Waypoint startPosition;
    private Waypoint depot;
    private Waypoint sample;
    private Waypoint park;
    private Waypoint release;


    public AutoPaths (GoldLocation location, StartLocation start) {
        this.location = location;
        this.start = start;

        if (start == StartLocation.CRATER) {
            startPosition = START_CRATER;
            depot = MARKER_CRATER;
            release = RELEASE_CRATER;
            park = PARK_CRATER;
            switch (location) {
                case LEFT:
                    sample = SAMPLE_LEFT_CRATER;
                    break;
                case CENTER:
                    sample = SAMPLE_CENTER_CRATER;
                    break;
                case RIGHT:
                    sample = SAMPLE_RIGHT_CRATER;
            }
        } else {
            startPosition = START_DEPOT;
            depot = MARKER_DEPOT;
            release = RELEASE_DEPOT;
            park = PARK_DEPOT;
            startPosition = START_DEPOT;
            switch (location) {
                case LEFT:
                    sample = SAMPLE_LEFT_DEPOT;
                    depot = MARKER_DEPOT_LEFT;
                    break;
                case CENTER:
                    sample = SAMPLE_CENTER_DEPOT;
                    break;
                case RIGHT:
                    sample = SAMPLE_RIGHT_DEPOT;
            }
        }
    }

    public ArrayList<Trajectory> paths () {
         if (start == StartLocation.DEPOT || location == GoldLocation.LEFT)
             return new TrajectoryBuilder(startPosition)
                     .to(release)
                     .to(sample)
                     .to(depot)
                     .turnTo(park.getHeading())
                     .to(park)
                     .build();
         else
             return new TrajectoryBuilder(startPosition)
                     .to(release)
                     .to(sample)
                     .to(CLEAR_ONE_CRATER)
                     .to(CLEAR_TWO_CRATER)
                     .to(depot)
                     .turnTo(park.getHeading())
                     .to(park)
                     .build();

    }

    public Waypoint start() {
        return startPosition;
    }


}
