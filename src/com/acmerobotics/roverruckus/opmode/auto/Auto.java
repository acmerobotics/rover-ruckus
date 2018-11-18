package com.acmerobotics.roverruckus.opmode.auto;

import com.acmerobotics.roverruckus.opmode.auto.paths.AutoPaths;
import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Auto {

    private Robot robot;
    private HardwareMap map;
    private OpMode opMode;
    private AutoPaths paths;

    public enum START_LOCATION {
        DEPOT,
        CRATER
    }
    private START_LOCATION start_location;
    private boolean latched;

    public Auto (OpMode opMode, HardwareMap map, START_LOCATION start_location, boolean latched) {
        this.opMode = opMode;
        this.map = map;
        this.start_location = start_location;
        this.latched = latched;
        this.paths = AutoPaths.getPaths(start_location);

        robot = new Robot(opMode, map);
    }

    public void run() {
        if (latched) {
            robot.lift.lower();
            robot.lift.waitForCompleteion();
        }

        robot.drive.followPath(paths.landerToSample());
        robot.drive.waitForCompleteion();

        robot.drive.followPath(paths.sampleToDepot());
        robot.drive.waitForCompleteion();
        robot.lift.markerDown();

        //dropOff

        robot.drive.followPath(paths.depotToPark());
        robot.drive.waitForCompleteion();

    }

}
