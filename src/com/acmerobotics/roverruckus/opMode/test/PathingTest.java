package com.acmerobotics.roverruckus.opMode.test;

import android.media.SoundPool;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.trajectory.Waypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "pathTest")
public class PathingTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);
        SoundPool pool = new SoundPool.Builder().build();
        int id = pool.load("/sdcard/tokyo_drift.mp33", 1);
        waitForStart();
        id = pool.play(id, 1, 1, 1, 0, 1);

//       Pose2d lander = new Pose2d(12, 12, Math.PI / 4);
//       Pose2d sample = new Pose2d(24, 48, -Math.PI / 4);
//       Pose2d depot = new Pose2d(48, 60, 0);
//       Pose2d crater = new Pose2d(-24, 60, 0);

        Waypoint lander = new Waypoint(new Pose2d(14.5, 14.5, Math.PI / 4), Math.PI / 4);
        Waypoint sample = new Waypoint(new Pose2d(24, 48, 3 * Math.PI / 4), Math.PI / 4);
        Waypoint depot = new Waypoint(new Pose2d(48, 60, Math.PI), Math.PI / 4, Math.PI);
        Waypoint almostThere = new Waypoint(new Pose2d(0, 622, -Math.PI / 2), Math.PI);
        Waypoint crater = new Waypoint(new Pose2d(-24, 62, -Math.PI / 2), Math.PI);


        robot.drive.setCurrentEstimatedPose(lander.pos());

//       SuperArrayList<Path> paths = new TrajectoryBuilder(lander).to(sample).to(depot).to(almostThere).to(crater).build();
//       SuperArrayList<Path> paths = new SuperArrayList<>();
//       paths.add(new PathBuilder(lander)
//               .splineTo(sample, new ConstantInterpolator(lander.getHeading()))
//               .splineTo(depot, new ConstantInterpolator(lander.getHeading())).build());
//       paths.add(new PathBuilder(depot).splineTo(crater, new ConstantInterpolator(lander.getHeading())).build());

//       for (Path path: paths) {
//           robot.drive.followTrajectory(path);
//           robot.waitForAllSubsystems();
//       }
//       pool.stop(id);
//       pool.unload(id);

    }

}
