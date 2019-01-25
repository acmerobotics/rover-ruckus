package com.acmerobotics.roverruckus.opMode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.robot.Placer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="rakeTest")
@Config
public class RakeTest extends LinearOpMode {

    public static double rakeUp = .65;
    public static double rakeDown = .2;

    @Override
    public void runOpMode() {
        DcMotor rakeMotor = hardwareMap.dcMotor.get("rakeMotor");
        rakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo rakeServo = hardwareMap.servo.get("rakeServo");
        Placer placer = new Placer(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            rakeMotor.setPower(gamepad1.left_stick_y);
//            if (gamepad1.left_bumper) placer.setIntakePower(1);
//            if (gamepad1.right_bumper) placer.setIntakePower(0);
            if (gamepad1.dpad_up) rakeServo.setPosition(rakeUp);
            else if (gamepad1.dpad_down) rakeServo.setPosition(rakeDown);
            if (gamepad1.a) placer.releaseFirst();
            else if (gamepad1.b) placer.releaseSecond();
            else if (gamepad1.x) placer.reset();
            TelemetryPacket packet = new TelemetryPacket();
            placer.update(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
