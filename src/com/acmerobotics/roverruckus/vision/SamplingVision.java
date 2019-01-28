package com.acmerobotics.roverruckus.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

//@Config
public class SamplingVision {

//    public static int MIN_HUE = 15;
//    public static int MIN_SAT = 150;
//    public static int MIN_VAL = 0;
//    public static int MAX_HUE = 30;
//    public static int MAX_SAT = 255;
//    public static int MAX_VAL = 255;
//    public static int K_SIZE = 20;
//    public static int BLUR_SIZE = 5;

    public static Mat processFrame (Mat in) {
        Mat frame = Mat.zeros(in.size(), in.type());
        Imgproc.cvtColor(in, frame, Imgproc.COLOR_RGB2HSV);
        Imgproc.blur(frame, frame, new Size(5,5));
        Core.inRange(frame, new Scalar(15, 150, 0), new Scalar(30, 255, 255), frame);
        Imgproc.morphologyEx(frame, frame, Imgproc.MORPH_CLOSE, Mat.ones(20, 20, frame.type()));
        return frame;
    }

}
