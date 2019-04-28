package com.acmerobotics.roverruckus.util;

import org.firstinspires.ftc.robotcontroller.internal.configuration.IntegerConfiguration;
import org.firstinspires.ftc.robotcontroller.internal.configuration.OpmodeConfiguration;

@OpmodeConfiguration
public class RoverRuckusConfiguration {

    public enum AllianceColor {
        RED,
        BLUE,
    }

    public AllianceColor color;

    public enum StartLocation {
        CRATER,
        DEPOT
    }

    public StartLocation startLocation;

    public boolean latched;

    public boolean sampleBoth;

    public boolean playMusic;

    @IntegerConfiguration(max=10)
    public int delay;

}
