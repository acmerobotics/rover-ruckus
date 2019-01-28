package com.acmerobotics.roverruckus.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Config
public class SamplingVision {

    public static int MIN_HUE = 15;
    public static int MIN_SAT = 150;
    public static int MIN_VAL = 0;
    public static int MAX_HUE = 30;
    public static int MAX_SAT = 255;
    public static int MAX_VAL = 255;
    public static int K_SIZE = 20;
    public static int BLUR_SIZE = 5;

    public static Mat processFrame (Mat in) {
        Mat frame = Mat.zeros(in.size(), in.type());
        Imgproc.cvtColor(in, frame, Imgproc.COLOR_RGB2HSV);
        Imgproc.blur(frame, frame, new Size(BLUR_SIZE,BLUR_SIZE));
        Core.inRange(frame, new Scalar(MIN_HUE, MIN_SAT, MIN_VAL), new Scalar(MAX_HUE, MAX_SAT, MAX_VAL), frame);
        Imgproc.morphologyEx(frame, frame, Imgproc.MORPH_CLOSE, Mat.ones(K_SIZE, K_SIZE, frame.type()));
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(in, contours, -1, new Scalar(0, 255, 0), 3);

        if (contours.size() >= 1) {
            RotatedRect best = null;
            double max = 0;
            for (MatOfPoint contour: contours) {
                MatOfPoint2f x = new MatOfPoint2f();
                x.fromArray(contour.toArray());
                RotatedRect rect = Imgproc.minAreaRect(x);
                double size = Math.hypot(rect.size.height, rect.size.width);
                if (size > max) {
                    max = size;
                    best = rect;
                }
            }
            Imgproc.rectangle(in, new Point(best.boundingRect().x, best.boundingRect().y), new Point(best.boundingRect().x + best.boundingRect().width, best.boundingRect().y + best.boundingRect().height), new Scalar(255, 0, 0), 5);
        }
        return in;
    }

}
