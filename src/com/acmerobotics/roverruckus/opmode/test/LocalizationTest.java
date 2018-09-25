package com.acmerobotics.roverruckus.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roverruckus.robot.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="localizationTest")
public class LocalizationTest extends LinearOpMode{

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();

        while (!isStopRequested()) {
            drive.setVelocity(new Pose2d(
                    -gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x
            ));

            drive.update();

            MecanumDrive.drawPose(packet.fieldOverlay(), drive.getCurrentEstimatedPose(), "black");
            packet.put("test", "test");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

}
