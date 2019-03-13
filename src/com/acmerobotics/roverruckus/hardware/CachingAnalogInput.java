package com.acmerobotics.roverruckus.hardware;

import com.acmerobotics.roverruckus.robot.Robot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.util.SerialNumber;

public class CachingAnalogInput extends AnalogInput {

    private Robot robot;
    private int hub, port;

    public CachingAnalogInput (Robot robot, int hub, int port) {
        super (new MockAnalogInputController(), 0);
        this.robot = robot;
        this.hub = hub;
        this.port = port;
    }

    @Override
    public double getVoltage() {
        return robot.getAnalogPort(hub, port);
    }

    private CachingAnalogInput(AnalogInputController controller, int channel) {
        super (controller, channel);
    }


}
