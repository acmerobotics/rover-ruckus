package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roverruckus.robot.Robot;
import com.acmerobotics.roverruckus.trajectory.TrajectoryBuilder;
import com.acmerobotics.roverruckus.trajectory.Waypoint;
import com.acmerobotics.roverruckus.util.SuperArrayList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="pathTest")
public class PathingTest extends LinearOpMode {

   @Override
   public void runOpMode() {
       Robot robot = new Robot(this, hardwareMap);

       waitForStart();

       Pose2d lander = new Pose2d(12, 12, Math.PI / 4);
       Pose2d sample = new Pose2d(24, 48, -Math.PI / 4);
       Pose2d depot = new Pose2d(48, 60, 0);
       Pose2d crater = new Pose2d(-24, 60, 0);
       robot.drive.setCurrentEstimatedPose(lander);

       SuperArrayList<Path> paths = new TrajectoryBuilder(new Waypoint(lander, lander.getHeading()))
               .to(new Waypoint(sample, lander.getHeading()))
               .to(new Waypoint(depot, lander.getHeading(), Math.PI))
               .to(new Waypoint(crater, Math.PI))
               .build();

       for (Path path: paths) {
           robot.drive.followPath(path);
           robot.waitForAllSubsystems();
       }

   }

}
