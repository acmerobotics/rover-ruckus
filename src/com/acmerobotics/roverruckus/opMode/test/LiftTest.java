package com.acmerobotics.roverruckus.opMode.test;

import android.os.Environment;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.FileWriter;

@Config
@TeleOp(name = "liftProfile")
public class LiftTest extends LinearOpMode {

    public static double distance = 20;


    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, hardwareMap);


        FileWriter writer = null;
        try {
            String path = Environment.getExternalStorageDirectory().getPath() + "/ACME/";
            new File(path).mkdirs();
            Log.e("path", path);
            writer = new FileWriter(path + "liftTest" + System.currentTimeMillis() + ".csv");
            writer.write("command, velocity\n");
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

//        robot.lift.disengageRatchet();
//        robot.lift.dumpMiddle();
//        robot.lift.placer.setEnabled(false);
//        robot.lift.setPosition(0);
//
        waitForStart();
//
        robot.lift.disengageRatchet();
//        robot.lift.goToPosition(distance);
//        robot.waitForAllSubsystems();
//        robot.lift.goToPosition(0);
//        robot.waitForAllSubsystems();


//        double direction = 1;
        double power = 0;
        double lastPosition = 0;
        double lastTime = System.currentTimeMillis();
        while (!isStopRequested()) {
           double pos = robot.lift.getPosition();
           double dx = pos - lastPosition;
           lastPosition = pos;
           double now = System.currentTimeMillis();
           double dt = (now - lastTime) / 1000.0;
           lastTime = now;
           double v = dx/dt;
           robot.addTelemetry("v", v);
           robot.addTelemetry("power", power);

           try {
               writer.write(power + ", " + v + "\n");
           } catch (Exception e) {
               Log.e("ahhh", "ahhhh");
               e.printStackTrace();
           }

//           if (pos >= 20) {
//               direction = -1;
//           } else if (pos <= 0) {
//               direction = 1;
//               power = 20;
//           }

           robot.lift.setVelocity(-gamepad2.left_stick_y);
           power = -gamepad2.left_stick_y;
           robot.update();
        }



        try {
            writer.flush();
            writer.close();
        } catch (Exception e) {

        }
    }


}
