package com.acmerobotics.roverruckus.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name="motorTest")
public class MotorControlTest extends LinearOpMode{

    public static double p;
    public static double i;
    public static double d;

    private static double pOld;
    private static double iOld;
    private static double dOld;

    public static double target = 0;
    public static double maxV = 13;


    @Override
    public void runOpMode() {
        FtcDashboard.start();
        TelemetryPacket packet = new TelemetryPacket();
        DcMotorControllerEx controller = (DcMotorControllerEx)
                hardwareMap.get(DcMotorController.class, "hub1");

        PIDCoefficients normal = controller.getPIDCoefficients(0, DcMotor.RunMode.RUN_USING_ENCODER);
        p = normal.p;
        i = normal.i;
        d = normal.d;

        controller.setMotorEnable(0);

        waitForStart();

        while(!isStopRequested()) {
            target = gamepad1.right_stick_y * maxV;
            packet.put("target", target);
            controller.setMotorVelocity(0, target, AngleUnit.RADIANS);

            double velocity = controller.getMotorVelocity(0, AngleUnit.RADIANS);
            packet.put("velocity", velocity);

            double error = velocity - target;
            packet.put("error", error);

            if (p != pOld || i != iOld || d != dOld) {
                pOld = p;
                iOld = i;
                dOld = d;

                controller.setPIDCoefficients(0, DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(p, i, d));
            }


            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        FtcDashboard.stop();
    }

}
