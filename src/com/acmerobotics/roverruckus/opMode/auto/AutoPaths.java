package com.acmerobotics.roverruckus.opMode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
    public static double RELEASE_THETA = -PI / 12;

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

    private static final Waypoint MARKER_CRATER = new Waypoint(new Pose2d(58, 48, 0), PI / 2, -PI / 2);
    private static final Waypoint MARKER_DEPOT = new Waypoint(new Pose2d(48, 64, -PI / 2), PI / 2, PI);
    private static final Waypoint MARKER_DEPOT_LEFT = new Waypoint(new Pose2d(48, 60, -PI / 2), 0, PI);

    private static final Waypoint PARK_CRATER = new Waypoint(new Pose2d(62, -15, -PI / 2),-PI/2);
    private static final Waypoint PARK_DEPOT = new Waypoint(new Pose2d(-8, 68, -PI), -PI);

    private static final Waypoint SAMPLE_LEFT_CRATER = new Waypoint(new Pose2d(36 + SAMPLE_DIST, -(12 + SAMPLE_DIST), PI / 4), -PI / 4, PI / 4);
    private static final Waypoint SAMPLE_CENTER_CRATER = new Waypoint(new Pose2d(24 + SAMPLE_DIST, -(24 + SAMPLE_DIST), PI / 4), -PI / 4, 3 * PI / 4);
    private static final Waypoint SAMPLE_RIGHT_CRATER = new Waypoint(new Pose2d(12 + SAMPLE_DIST, -(36 + SAMPLE_DIST), PI / 4), -PI / 4, 3 * PI / 4);

    private static final Waypoint SAMPLE_LEFT_SECOND = new Waypoint(new Pose2d(36 - SAMPLE_DIST, 60 - SAMPLE_DIST, -PI / 4), -3 * PI / 4, PI / 4);
    private static final Waypoint SAMPLE_CENTER_SECOND = new Waypoint(new Pose2d(48 - SAMPLE_DIST, 48 - SAMPLE_DIST, -PI / 4), -3 * PI / 4, PI / 4);
    private static final Waypoint SAMPLE_RIGHT_SECOND = new Waypoint(new Pose2d(60 - SAMPLE_DIST, 36 - SAMPLE_DIST, -PI / 4), -3 * PI / 4, PI / 4);

    private static final Waypoint SAMPLE_LEFT_DEPOT = new Waypoint(new Pose2d(24, 48, -PI / 4), PI / 4);
    private static final Waypoint SAMPLE_CENTER_DEPOT = new Waypoint(new Pose2d(36, 36, -PI / 4), PI / 3);
    private static final Waypoint SAMPLE_RIGHT_DEPOT = new Waypoint(new Pose2d(48, 24, -PI / 4), PI / 4);

    private static final Waypoint CLEAR_ONE_CRATER = new Waypoint(new Pose2d(36, -16, PI / 4), PI / 5);
    private static final Waypoint CLEAR_TWO_CRATER = new Waypoint(new Pose2d(48, 0, 0), PI/4);

    private GoldLocation location;
    private StartLocation start;
    private boolean sampleBoth;
    private Waypoint startPosition;
    private Waypoint depot;
    private Waypoint firstDepot;
    private Waypoint sample;
    private Waypoint park;
    private Waypoint release;
    private Waypoint sampleSecond;
    private Waypoint clearOne;
    private Waypoint clearTwo;


    public AutoPaths(GoldLocation location, StartLocation start, boolean sampleBoth) {
        this.location = location;
        this.start = start;
        this.sampleBoth = (sampleBoth || location == GoldLocation.RIGHT) && start == StartLocation.CRATER;

        clearOne = CLEAR_ONE_CRATER;
        clearTwo = CLEAR_TWO_CRATER;

        if (start == StartLocation.CRATER) {
            startPosition = START_CRATER;
            depot = MARKER_CRATER;
            release = RELEASE_CRATER;
            park = PARK_CRATER;
            switch (location) {
                case LEFT:
                    sample = SAMPLE_LEFT_CRATER;
                    sampleSecond = SAMPLE_LEFT_SECOND;
                    break;
                case CENTER:
                    sample = SAMPLE_CENTER_CRATER;
                    sampleSecond = SAMPLE_CENTER_SECOND;
                    break;
                case RIGHT:
                    sample = SAMPLE_RIGHT_CRATER;
                    sampleSecond = SAMPLE_RIGHT_SECOND;
            }
        } else {
            startPosition = START_DEPOT;
            depot = MARKER_DEPOT;
            release = RELEASE_DEPOT;
            park = PARK_DEPOT;
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

        if (sampleBoth) {
            firstDepot = new Waypoint(depot.pos(), depot.getEnter().getHeading(), location == GoldLocation.RIGHT ? -PI / 2 : PI);
            depot = new Waypoint(depot.pos(), 0, depot.getExit().getHeading());
            if (location == GoldLocation.RIGHT)
                clearTwo = new Waypoint(new Pose2d(SAMPLE_RIGHT_DEPOT.pos().pos().plus(new Vector2d(6, -6)), 0), PI/2);
        }

    }

   public ArrayList<Trajectory> paths() {
        TrajectoryBuilder builder = new TrajectoryBuilder(startPosition)
                .to(release)
                .addFlag(AutoFlag.LOWER_LIFT)
                .to(sample);
        if (start == StartLocation.CRATER && location != GoldLocation.LEFT)
            builder
                    .to(clearOne)
                    .to(clearTwo);

        if (sampleBoth && location != GoldLocation.RIGHT)
            builder
                    .to(firstDepot)
                    .to(sampleSecond);

        builder.to(depot);

        if (start == StartLocation.CRATER)
            builder.turnTo(-PI/3);

        builder
                .addFlag(AutoFlag.RELEASE_MARKER)
                .turnTo(park.getHeading())
                .to(park);

        return builder.build();

    }

    public Waypoint start() {
        return startPosition;
    }


}
