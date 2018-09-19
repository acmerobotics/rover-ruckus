package com.acmerobotics.roverruckus.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="RakeTest")
public class RakeTest extends OpMode{

    private DcMotor extend;
    private Servo wrist;
    private long lastTime;
    private double wristPosition = 0;

    public static double wristSpeed = .0005; //per millis

    @Override
    public void init() {
        extend = hardwareMap.get(DcMotor.class, "extend");
        wrist = hardwareMap.get(Servo.class, "wrist");
        lastTime = System.currentTimeMillis();
    }



    @Override
    public void loop() {
        extend.setPower(gamepad1.left_stick_y);

        long now = System.currentTimeMillis();
        long dt = lastTime - now;
        lastTime = now;

        wristPosition += wristSpeed * dt * gamepad1.right_stick_y;
        wrist.setPosition(wristPosition);

    }

}
