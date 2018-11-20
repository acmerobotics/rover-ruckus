package com.acmerobotics.roverruckus.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="craterLanded")
public class CraterLanded extends LinearOpMode implements Running {

    public void runOpMode() {
        Auto auto = new Auto(this, this, hardwareMap, Auto.START_LOCATION.CRATER, false);
        waitForStart();
        auto.run();
    }

    public boolean running() {
        return opModeIsActive();
    }

}
