package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.robot.Placer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "placerTest")
public class PlacerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Placer place = new Placer(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.a) place.releaseFirst();
            if (gamepad1.b) place.releaseSecond();
            if (gamepad1.x) place.reset();
            TelemetryPacket packet = new TelemetryPacket();
            place.update(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

}
