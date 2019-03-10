package com.acmerobotics.roverruckus.vision;

import org.opencv.core.Mat;

public interface Tracker {

    void processFrame (Mat frame);

}
