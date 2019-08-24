package com.acmerobotics.roverruckus.opMode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roverruckus.trajectory.Trajectory;
import com.acmerobotics.roverruckus.trajectory.TrajectoryBuilder;
import com.acmerobotics.roverruckus.trajectory.Waypoint;
import com.acmerobotics.roverruckus.util.RoverRuckusConfiguration;
import com.acmerobotics.roverruckus.util.SuperArrayList;
import com.acmerobotics.roverruckus.vision.GoldLocation;

import static com.acmerobotics.roverruckus.util.RoverRuckusConfiguration.StartLocation.CRATER;
import static com.acmerobotics.roverruckus.util.RoverRuckusConfiguration.StartLocation.DEPOT;


@Config
public class AutoPaths {

    private static double PI = Math.PI;
    private static double SAMPLE_DIST = 1;
    private static double INTAKE_DIST = 10;
    public static double START_DIST = 19 / Math.sqrt(2);
    public static double RELEASE_X = 2;
    public static double RELEASE_Y = 1;
    public static double RELEASE_THETA = -PI / 12;
    public static double SCORE_X = 4;
    public static double SCORE_Y = 6;

    public static final Vector2d RAKE_POSITION_CRATER = new Vector2d(52, 44);
    public static final Vector2d RAKE_POSITION_DEPOT = new Vector2d(60, 60);

    private static final Pose2d RELEASE = new Pose2d(RELEASE_X, RELEASE_Y, RELEASE_THETA);

    private static final Waypoint START_CRATER = new Waypoint(new Pose2d(START_DIST, -START_DIST, -PI / 4), PI / 4);
    private static final Waypoint START_DEPOT = new Waypoint(new Pose2d(START_DIST, START_DIST, PI / 4), 3 * PI / 4);

    private static final Waypoint RELEASE_CRATER = new Waypoint(new Pose2d(START_CRATER.pos().pos()
            .plus(RELEASE.pos().rotated(START_CRATER.pos().getHeading())),
            -PI / 3),
            0, 0);

    private static final Waypoint RELEASE_DEPOT = new Waypoint(new Pose2d(START_DEPOT.pos().pos()
            .plus(RELEASE.pos().rotated(START_DEPOT.pos().getHeading())),
            PI / 6),
            PI / 2, PI / 4);

    public Waypoint score () {
        if (start == CRATER)
            return new Waypoint(new Pose2d(START_CRATER.pos().pos()
            .plus(new Vector2d(SCORE_X, SCORE_Y).rotated(START_CRATER.getHeading())),
                    -7*PI/24),
                    3*PI/4,
                    -PI/4);
        return START_DEPOT;
    }

    private static final Waypoint MARKER_CRATER = new Waypoint(new Pose2d(54, 6, PI/2), PI / 2, -3 * PI / 4);
    private static final Waypoint MARKER_DEPOT = new Waypoint(new Pose2d(48, 60, -PI / 2), PI / 2, PI);

    private static final Waypoint PARK_CRATER = new Waypoint(new Pose2d(60, -17, -PI / 2),-PI/2);
    private static final Waypoint PARK_DEPOT = new Waypoint(new Pose2d(-14, 61, PI), PI);
    private static final Waypoint PARK_CLEAR_ONE = new Waypoint(new Pose2d(12, 36, -PI/4), 3*PI/4);
    private static final Waypoint PARK_CLEAR_TWO = new Waypoint(new Pose2d(0, 60, -PI/2), PI/2, PI);

//    private static final Waypoint SAMPLE_LEFT_DEPOT = new Waypoint(new Pose2d(24, 48, PI / 4), PI / 4);
//    private static final Waypoint SAMPLE_CENTER_DEPOT = new Waypoint(new Pose2d(36, 36, PI / 4), PI / 4);
//    private static final Waypoint SAMPLE_RIGHT_DEPOT = new Waypoint(new Pose2d(48, 24, PI / 4), PI / 4);

    private static final Waypoint SAMPLE_LEFT_DEPOT= new Waypoint(new Pose2d(18, 42, PI / 6), 0, 3*PI/4);
    private static final Waypoint SAMPLE_CENTER_DEPOT = new Waypoint(new Pose2d(30, 30, PI / 3), PI / 4, -3*PI/4);
    private static final Waypoint SAMPLE_RIGHT_DEPOT= new Waypoint(new Pose2d(42, 18, PI/2), PI / 2, -3*PI/4);

    private static final Waypoint SAMPLE_BOTH_LEFT = new Waypoint(new Pose2d(24, 48, PI / 2), PI, 0);
    private static final Waypoint SAMPLE_BOTH_CENTER = new Waypoint(new Pose2d(36, 36, PI / 2), PI, 0);
    private static final Waypoint SAMPLE_BOTH_RIGHT = new Waypoint(new Pose2d(48, 16, PI/2), PI/2, -PI/2);


    private static final Waypoint SAMPLE_SECORD_CLEAR = new Waypoint(new Pose2d(60, 24, PI/2), PI/2);
    private static final Waypoint SAMPLE_SECOND_RETURN = new Waypoint(new Pose2d(60, 24, PI/2), -PI/2, true);
    private static final Waypoint SAMPLE_SECOND_RETURN_SECOND = new Waypoint(new Pose2d(12, -12, PI/4), -3*PI/4);

    private static final Waypoint CLEAR_ONE_CRATER = new Waypoint(new Pose2d(36, -16, -PI / 2), PI / 5);
    private static final Waypoint CLEAR_TWO_CRATER = new Waypoint(new Pose2d(52, 0, -PI/2), PI/4);

    private Waypoint lastPosition;

    private GoldLocation location;
    private RoverRuckusConfiguration.StartLocation start;
    private AutoOpMode opMode = null;

    public AutoPaths(AutoOpMode opMode, GoldLocation location, RoverRuckusConfiguration.StartLocation start) {
        this.location = location;
        this.start = start;
        this.lastPosition = start();
        this.opMode = opMode;
    }

    public AutoPaths (GoldLocation location, RoverRuckusConfiguration.StartLocation start) {
        this.location = location;
        this.start = start;
        this.lastPosition = start();
    }

    public Waypoint MINERAL_OFFSET_LEFT_CRATER (double offset) {
        return new Waypoint(new Pose2d(36 + offset, -(12 + offset), -PI / 4), -PI / 4, PI / 4);
    }

    public Waypoint MINERAL_OFFSET_CENTER_CRATER (double offset) {
        return new Waypoint(new Pose2d(24 + offset, -(24 + offset), -PI / 4), -PI / 4, 3 * PI / 4);
    }
    public Waypoint MINERAL_OFFSET_RIGHT_CRARTER (double offset) {
        return new Waypoint(new Pose2d(12 + offset, -(36 + offset), -PI / 4), -PI / 4, 3 * PI / 4);
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
            case RIGHT: return start == CRATER ? MINERAL_OFFSET_RIGHT_CRARTER(SAMPLE_DIST) : SAMPLE_RIGHT_DEPOT;
            case CENTER: return start == CRATER ? MINERAL_OFFSET_CENTER_CRATER(SAMPLE_DIST): SAMPLE_CENTER_DEPOT;
            default: return start == CRATER ? MINERAL_OFFSET_LEFT_CRATER(SAMPLE_DIST): SAMPLE_LEFT_DEPOT;
        }
    }

    public Waypoint sampleSecond () {
        switch (location) {
            case RIGHT: return SAMPLE_BOTH_RIGHT;
            case CENTER: return SAMPLE_BOTH_CENTER;
            default: return SAMPLE_BOTH_LEFT;
        }
    }

    public Waypoint intake() {
        switch (location) {
            case RIGHT: return start == CRATER ? MINERAL_OFFSET_RIGHT_CRARTER(INTAKE_DIST) : SAMPLE_RIGHT_DEPOT;
            case CENTER: return start == CRATER ? MINERAL_OFFSET_CENTER_CRATER(INTAKE_DIST): SAMPLE_CENTER_DEPOT;
            default: return start == CRATER ? MINERAL_OFFSET_LEFT_CRATER(INTAKE_DIST): SAMPLE_LEFT_DEPOT;
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

    public SuperArrayList<Trajectory> toSampleCrater () {
        TrajectoryBuilder builder = getBuilder();
        lastPosition = sample();
        return builder.to(sample()).addAction(-2, opMode.groundIntake).build();
    }

    public SuperArrayList<Trajectory> toSampleDepot () {
        TrajectoryBuilder builder = getBuilder();
        lastPosition = sample();
        return builder.to(sample()).addAction(-1, opMode.extendRake).addAction(-.5, opMode.groundIntake).build();
    }

    public SuperArrayList<Trajectory> toSampleBoth () {
        TrajectoryBuilder builder = getBuilder();
        if (location == GoldLocation.RIGHT) {
            builder.to(sampleSecond()).addActionOnCompletion(opMode.groundIntake).addActionOnCompletion(opMode.extendRake);
            lastPosition = sampleSecond();
        }
        else {
            builder.to(SAMPLE_SECORD_CLEAR).to(sampleSecond()).to(SAMPLE_SECOND_RETURN).addActionOnCompletion(opMode.extendRake);
            lastPosition = SAMPLE_SECOND_RETURN;
        }
        return builder.build();
    }

    public SuperArrayList<Trajectory> toMarker () {
        TrajectoryBuilder builder = new TrajectoryBuilder(release());
        lastPosition = marker();
        return builder.to(marker()).addAction(-2, opMode.extendRake).build();
    }

    public SuperArrayList<Trajectory> toScore () {
        TrajectoryBuilder builder = getBuilder();
        lastPosition = start();
        return builder.to(score()).addActionOnStart(opMode.reverseIntake).addActionOnStart(opMode.extendRake).addActionOnCompletion(opMode.stopIntake).addActionOnStart(opMode.raiseLift).build();
    }

    public SuperArrayList<Trajectory> toIntake () {
        TrajectoryBuilder builder = getBuilder();
        lastPosition = intake();
        return builder.to(intake())
                .addActionOnStart(opMode.lowerLift)
                .addActionOnStart(opMode.blockingIntake)
                .build();
    }

    public SuperArrayList<Trajectory> toPark () {
        TrajectoryBuilder builder = getBuilder();
        lastPosition = park();
        if (location != GoldLocation.LEFT) builder.to(PARK_CLEAR_ONE);
        builder.to(PARK_CLEAR_TWO).addActionOnCompletion(opMode.stopIntake);
        builder.turnTo(PI);
        builder.to(park());
        return builder.build();
    }

    public SuperArrayList<Trajectory> mockDoubleSample () {
        start = CRATER;
        TrajectoryBuilder builder = new TrajectoryBuilder(start());
        builder.to(release());
        if (location == GoldLocation.RIGHT) builder.to(sampleSecond());
        else builder.to(SAMPLE_SECORD_CLEAR).to(sampleSecond()).to(SAMPLE_SECOND_RETURN);
        builder.to(sample()).to(intake());
        return builder.build();
    }

    public SuperArrayList<Trajectory> mockDepot () {
        start = DEPOT;
        TrajectoryBuilder builder = new TrajectoryBuilder(start());
        builder.to(release());
        builder.to(sample());
        if (location != GoldLocation.LEFT) builder.to(PARK_CLEAR_ONE);
        builder.to(PARK_CLEAR_TWO).to(park());
        return builder.build();
    }

    public SuperArrayList<Trajectory> mockCycle () {
        start = CRATER;
        TrajectoryBuilder builder = new TrajectoryBuilder(start());
        builder.to(release());
        builder.to(marker());
        builder.to(sample());
        builder.to(intake());
        builder.to(score());
        return builder.build();
    }

    private TrajectoryBuilder getBuilder () {
        return new TrajectoryBuilder(new Waypoint(opMode.robot.drive.getCurrentEstimatedPose(), lastPosition.getExit().getHeading()));
    }

}
