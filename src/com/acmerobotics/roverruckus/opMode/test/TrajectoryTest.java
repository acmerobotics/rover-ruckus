package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roverruckus.trajectory.Trajectory;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;

public class TrajectoryTest {
    public static void main(String[] args) throws FileNotFoundException, IOException {
        Trajectory trajectory = new Trajectory(new PathBuilder(new Pose2d()).lineTo(new Vector2d(48, 0), new LinearInterpolator(0, Math.PI)).build());
        FileWriter writer = new FileWriter("trajectory.csv");
//        writer.write("vx, vy, omega");
        for (double t = 0; !trajectory.isComplete(); t += .01) {
            Pose2d update = trajectory.update(t, new Pose2d(), new TelemetryPacket());
            writer.write(String.format("%f, %f, %f, %f \n", t, update.getX(), update.getY(), update.getHeading()));

        }
        writer.flush();
    }
}
