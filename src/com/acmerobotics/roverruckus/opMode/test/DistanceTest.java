package com.acmerobotics.roverruckus.opMode.test;

import android.os.Environment;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.hardware.SharpDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.io.File;
import java.io.FileWriter;

@Autonomous(name="distanceTest")
public class DistanceTest extends LinearOpMode {

    @Override
    public void runOpMode () {
        SharpDistanceSensor sensor = new SharpDistanceSensor(hardwareMap.analogInput.get("dist"));
//        AnalogInput linear = hardwareMap.analogInput.get("pot");
        TelemetryPacket packet;


//        FileWriter writer;
//        try {
//            String path = Environment.getExternalStorageDirectory().getPath() + "/ACME/";
//            new File(path).mkdirs();
//            Log.e("path", path);
//            writer = new FileWriter(path + "linearization" + System.currentTimeMillis() + ".csv");
//            writer.write("dist, pot\n");
//        } catch (Exception e) {
//            e.printStackTrace();
//            return;
//        }

        waitForStart();

        while (!isStopRequested()) {
            double dist = sensor.getUnscaledDistance();
//            double pot = linear.getVoltage();
            packet = new TelemetryPacket();
            packet.put("dist", dist);
//            packet.put("pot", pot);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

//           try {
//               writer.write(dist + ", " + pot + "\n");
//           } catch (Exception e) {
//               Log.e("ahhh", "ahhhh");
//               e.printStackTrace();
//           }
//        }


//        try {
//            writer.flush();
//            writer.close();
//        } catch (Exception e) {
//
        }
    }

}
