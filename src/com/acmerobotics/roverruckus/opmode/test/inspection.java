package com.acmerobotics.roverruckus.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="inspection")
@Disabled
public class inspection extends LinearOpMode {
//    DcMotor motor = hardwareMap.dcMotor.get("intakeMotor");

    public void runOpMode() {
        DcMotor motor = hardwareMap.dcMotor.get("intakeMotor");
        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y);
        }
    }

}
