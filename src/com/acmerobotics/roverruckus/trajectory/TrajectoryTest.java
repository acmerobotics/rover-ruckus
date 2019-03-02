package com.acmerobotics.roverruckus.trajectory;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.ParametricCurve;
import com.acmerobotics.roverruckus.opMode.auto.AutoPaths;
import com.acmerobotics.roverruckus.trajectory.Trajectory;
import com.acmerobotics.roverruckus.vision.GoldLocation;

import org.firstinspires.ftc.robotcontroller.internal.configuration.StartLocation;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryTest {
    @SuppressLint("DefaultLocale")
    public static void main(String[] args) throws IOException {
        for (boolean sampleBoth : Arrays.asList(true, false))
            for (GoldLocation loc : GoldLocation.values())
                for (StartLocation start : StartLocation.values()) {
                    if (sampleBoth && start == StartLocation.DEPOT) continue;
                    String name = String.format("./out/paths/%s_%s_%s.csv", start, loc, sampleBoth);
                    System.out.println(name);
                    File outputDir = new File("./out/paths");
                    outputDir.mkdirs();
                    FileWriter writer = new FileWriter(name);

                    ArrayList<Trajectory> trajectories = new AutoPaths(loc, start, sampleBoth).paths();
                    for (Trajectory trajectory: trajectories ) {
                        if (trajectory.getPath() == null) continue;
                        List<ParametricCurve> curves = trajectory.getPath().getParametricCurves();
                        for (ParametricCurve curve: curves) {
                            for (double s = 0; s <= curve.length(); s += 1) {
//                                System.out.println (curve.internal)
                            }
                        }
                    }
                    for (Trajectory trajectory : trajectories) {
                        for (double t = 0; t <= trajectory.duration(); t += .01) {
                            Pose2d pose = trajectory.getPose(t);
                            double v = trajectory.getV(t);
                            writer.write(String.format("%f, %f, %f, %f\n", pose.getX(), pose.getY(), pose.getHeading(), v));
                        }
                        writer.write("\n");
                    }
                    writer.flush();
                    writer.close();
                }
    }
}
