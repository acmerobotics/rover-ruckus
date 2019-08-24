package com.acmerobotics.roverruckus.util;


import com.acmerobotics.lib.config.IntegerConfiguration;
import com.acmerobotics.lib.config.OpmodeConfiguration;

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
