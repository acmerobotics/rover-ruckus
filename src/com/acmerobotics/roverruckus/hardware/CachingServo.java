package com.acmerobotics.roverruckus.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class CachingServo implements Servo, CachingMotor {

    private Servo delegate;
    private double cachedPosition = 0;
    private boolean needsUpdate = false;

    public CachingServo(Servo delegate) {
        this.delegate = delegate;
    }

    @Override
    public synchronized void update() {
        synchronized (this) {
            if (needsUpdate) delegate.setPosition(cachedPosition);
        }
    }

    @Override
    public synchronized ServoController getController() {
        return delegate.getController();
    }

    @Override
    public synchronized int getPortNumber() {
        return delegate.getPortNumber();
    }

    @Override
    public synchronized void setDirection(Direction direction) {
        delegate.setDirection(direction);
    }

    @Override
    public synchronized Direction getDirection() {
        return delegate.getDirection();
    }

    @Override
    public void setPosition(double position) {
        synchronized (this) {
            if (position != cachedPosition) {
                cachedPosition = position;
                needsUpdate = true;
            }
        }
    }

    @Override
    public synchronized double getPosition() {
        return delegate.getPosition();
    }

    @Override
    public synchronized void scaleRange(double min, double max) {
        delegate.scaleRange(min, max);
    }

    @Override
    public synchronized Manufacturer getManufacturer() {
        return delegate.getManufacturer();
    }

    @Override
    public synchronized String getDeviceName() {
        return delegate.getDeviceName();
    }

    @Override
    public synchronized String getConnectionInfo() {
        return delegate.getConnectionInfo();
    }

    @Override
    public synchronized int getVersion() {
        return delegate.getVersion();
    }

    @Override
    public synchronized void resetDeviceConfigurationForOpMode() {
        delegate.resetDeviceConfigurationForOpMode();
    }

    @Override
    public synchronized void close() {
        delegate.close();
    }
}
