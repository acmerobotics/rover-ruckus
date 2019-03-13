package com.acmerobotics.roverruckus.hardware;

import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.util.SerialNumber;

public class MockAnalogInputController implements AnalogInputController {
    @Override
    public double getAnalogInputVoltage(int channel) {
        return 0;
    }

    @Override
    public double getMaxAnalogInputVoltage() {
        return 0;
    }

    @Override
    public SerialNumber getSerialNumber() {
        return null;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
