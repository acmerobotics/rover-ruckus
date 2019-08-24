package com.acmerobotics.roverruckus.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.lib.config.ConfigurationLoader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="configTest")
public class ConfigTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        RoverRuckusConfiguration configuration = (RoverRuckusConfiguration) new ConfigurationLoader(hardwareMap.appContext).getConfig();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("color", configuration.color);
        packet.put("start", configuration.startLocation);
        packet.put("delay", configuration.delay);
        packet.put("both", configuration.sampleBoth);
        packet.put("latched", configuration.latched);
        packet.put("music", configuration.playMusic);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        waitForStart();
    }
}
