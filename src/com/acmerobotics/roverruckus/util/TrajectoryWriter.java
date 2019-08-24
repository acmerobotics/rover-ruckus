package com.acmerobotics.roverruckus.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roverruckus.opMode.auto.AutoPaths;
import com.acmerobotics.roverruckus.trajectory.Trajectory;
import com.acmerobotics.roverruckus.vision.GoldLocation;

import java.io.FileWriter;

public class TrajectoryWriter {

    public static void main (String[] args) {
        AutoPaths paths = new AutoPaths(GoldLocation.CENTER, RoverRuckusConfiguration.StartLocation.CRATER);
        writeTrajectory(paths.mockCycle(), "double");
        paths = new AutoPaths(GoldLocation.CENTER, RoverRuckusConfiguration.StartLocation.DEPOT);
        writeTrajectory(paths.mockDepot(), "depot");
    }


    private static void writeTrajectory (SuperArrayList<Trajectory> trajectories, String name) {
        try {
            FileWriter writer = new FileWriter("out/paths/" + name + ".csv");
            for (Trajectory trajectory: trajectories) {
                for (double t = 0; t <= trajectory.duration(); t += .01) {
                    Pose2d pose = trajectory.getPose(t);
                    double v = trajectory.getV(t);
                    writer.write(pose.getX() + ", " + pose.getY() + ", " + pose.getHeading() + ", " + v + "\n");
                }
                writer.write('\n');
            }
            writer.flush();
            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
