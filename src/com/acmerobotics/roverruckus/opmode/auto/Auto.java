package com.acmerobotics.roverruckus.opmode.auto;

import com.acmerobotics.roverruckus.opmode.auto.paths.AutoPaths;
import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Auto {

    private Robot robot;
    private HardwareMap map;
    private OpMode opMode;
    private Running running;
    private AutoPaths paths;

    public enum START_LOCATION {
        DEPOT,
        CRATER
    }
    private START_LOCATION start_location;
    private boolean latched;

    public Auto (OpMode opMode, Running running, HardwareMap map, START_LOCATION start_location, boolean latched) {
        this.opMode = opMode;
        this.running = running;
        this.map = map;
        this.start_location = start_location;
        this.latched = latched;
        this.paths = AutoPaths.getPaths(start_location);

        robot = new Robot(opMode, map);
    }

    public void run(){
        if (latched) {
            robot.lift.lower();
            robot.update();
            waitForPathCompletion();
        }
        robot.drive.setCurrentEstimatedPose(paths.lander);

        robot.drive.followPath(paths.landerToDepot());
        waitForPathCompletion();

        robot.lift.markerDown();

        //dropOff

        robot.drive.followPath(paths.depotToPark());
        waitForPathCompletion();

    }

    public void waitForPathCompletion() {
        while (running.running() && robot.drive.isFollowingPath()) {
            robot.update();
        }
    }

}
