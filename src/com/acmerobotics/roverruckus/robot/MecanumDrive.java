package com.acmerobotics.roverruckus.robot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.drive.Kinematics;
import com.acmerobotics.roadrunner.drive.MecanumKinematics;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roverruckus.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

public class MecanumDrive{

    private DcMotorEx[] motors;
    private static final String[] motorNames = {
            "m0",
            "m1",
            "m2",
            "m3"
    };
    private static final Vector2d[] wheelPositions = {
            new Vector2d(1,2),
            new Vector2d(-1, 2),
            new Vector2d(-1,-2),
            new Vector2d(1, -2)
    };
    private static final Vector2d[] rotorDirections = {
            new Vector2d(1,-1),
            new Vector2d(1, 1),
            new Vector2d(1, -1),
            new Vector2d(1, 1)
    };
    private static final double radius = 2;

    public static double axialMaxV = 1;
    public static double axialMaxA = 1;
    public static double axialMaxJ = 1;

    public static double headingMaxV = 1;
    public static double headingMaxA = 1;
    public static double headingMaxJ = 1;

    private Pose2d targetVelocity = new Pose2d(0,0,0);

    private Pose2d currentEstimatedPose = new Pose2d(0, 0, 0);
    private boolean estimatingPose = true;
    private double[] lastWheelPositions = new double[4];
    private long lastUpdate = 0;

    private Trajectory trajectory;
    private long startTime;

    private enum Mode {
        OPEN_LOOP,
        FOLLOWING_PATH
    }

    private Mode currentMode = Mode.OPEN_LOOP;

    public MecanumDrive(HardwareMap hardwareMap) {
        motors = new DcMotorEx[motorNames.length];
        for (int i = 0; i < motorNames.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, motorNames[i]);
        }
        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);

    }

    private void internalSetVelocity (Pose2d v) {
        for (int i = 0; i < motors.length; i++) {
            Vector2d rotorVelocity = new Vector2d(
              v.getX() - v.getHeading() * wheelPositions[i].getY(),
              v.getY() + v.getHeading() * wheelPositions[i].getX()
            );
            double surfaceVelocity = rotorVelocity.dot(rotorDirections[i]);
            double wheelVelocity = surfaceVelocity / radius;
            motors[i].setVelocity(wheelVelocity, AngleUnit.RADIANS);
        }
    }

    /**
     * Set the robot velocity for open-loop control
     *
     * @param target Desired velocity of the robot, on [-1, 1], will be scaled to max V
     */
    public void setVelocity (Pose2d target) {
        double v = target.pos().norm();
        v = Range.clip(v, -1, 1) * axialMaxV;
        double theta = Math.atan2(target.getX(), target.getY());
        double omega = target.getHeading() * headingMaxV;

        targetVelocity = new Pose2d(v * Math.cos(theta), v * Math.sin(theta), omega);
    }

    public void followPath (Path path) {
        trajectory = new Trajectory(path);
        currentMode = Mode.FOLLOWING_PATH;
        startTime = System.currentTimeMillis();
    }

    public boolean isFollowingPath () {
        return !trajectory.isComplete() && currentMode == Mode.FOLLOWING_PATH;
    }

    public Pose2d getCurrentEstimatedPose() {
        return currentEstimatedPose;
    }

    public void stop() {
        targetVelocity = new Pose2d(0,0,0);
        currentMode = Mode.OPEN_LOOP;
    }


    public void update() {
        if (estimatingPose) updatePoseEstimate();

        switch (currentMode) {
            case OPEN_LOOP:
                internalSetVelocity(targetVelocity);
                break;

//            case FOLLOWING_PATH:
//                internalSetVelocity(Kinematics.fieldToRobotPoseVelocity(currentEstimatedPose, trajectory.update(System.currentTimeMillis() - startTime, currentEstimatedPose)));
//                if (trajectory.isComplete()) currentMode = Mode.OPEN_LOOP;
//                break;

        }
    }

    private void updatePoseEstimate() {
        if (lastUpdate == 0) {
            lastUpdate = System.nanoTime();
            for (int i = 0; i < 4; i++) {
                lastWheelPositions[i] = motors[i].getCurrentPosition();
            }
            return;
        }

        long now = System.nanoTime();
        long dt = now - lastUpdate;
        lastUpdate = now;

        List<Double> wheelVelocities = new ArrayList<>(4);
        for (int i = 0; i < 4; i++) {
            double pos = motors[i].getCurrentPosition();
            wheelVelocities.set(i, (pos - lastWheelPositions[i]) / dt);
            lastWheelPositions[i] = pos;
        }
        Pose2d v = MecanumKinematics.wheelToRobotVelocities(wheelVelocities, 1, 1);
        Pose2d delta = v.times(dt);
        currentEstimatedPose = Kinematics.relativeOdometryUpdate(currentEstimatedPose, delta);
    }

    public static void drawPose (Canvas overlay, Pose2d pose, String color) {
        overlay.setStroke(color);
        overlay.strokeRect(pose.getX() - 9, pose.getY() - 9, 18, 18);
        overlay.strokeLine(pose.getX(), pose.getY(), pose.getX() + 9, pose.getY());
    }

}
