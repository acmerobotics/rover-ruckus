package com.acmerobotics.roverruckus.hardware;

public interface CachingSensor {

    void setEnabled(boolean enabled);

    boolean updated();

    void update();

}
