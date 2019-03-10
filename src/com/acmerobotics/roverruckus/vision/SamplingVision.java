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
public class SamplingVision implements Tracker{

    public static int MIN_HUE = 5;
    public static int MIN_SAT = 150;
    public static int MIN_VAL = 0;
    public static int MAX_HUE = 30;
    public static int MAX_SAT = 255;
    public static int MAX_VAL = 255;
    public static int K_SIZE = 20;
    public static int BLUR_SIZE = 5;
    public static int DISPLAY = 0;
    public static int THRESHOLD = 250;
    public static int CROP = 125;
    private static int i = 0;
    private static GoldLocation location = GoldLocation.CENTER;

    private double x = 0;
    private double y = 0;
    private boolean enabled = false;

    public synchronized double getX() {
        return x;
    }

    public synchronized double getY() {
        return y;
    }

    public synchronized void processFrame(Mat in) {
        if (!enabled) return;
        Size s = new Size(600, 800);
        Mat frame = Mat.zeros(s, in.type());
        Imgproc.resize(in, frame, s);

        Imgproc.blur(frame, frame, new Size(BLUR_SIZE, BLUR_SIZE));
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
        Core.inRange(frame, new Scalar(MIN_HUE, MIN_SAT, MIN_VAL), new Scalar(MAX_HUE, MAX_SAT, MAX_VAL), frame);
        Imgproc.morphologyEx(frame, frame, Imgproc.MORPH_OPEN, Mat.ones(K_SIZE, K_SIZE, frame.type()));
        Imgproc.morphologyEx(frame, frame, Imgproc.MORPH_CLOSE, Mat.ones(K_SIZE, K_SIZE, frame.type()));

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(in, contours, -1, new Scalar(0, 255, 0), 3);

        if (contours.size() >= 1) {
            RotatedRect best = null;
            double max = 0;
            for (MatOfPoint contour : contours) {
                MatOfPoint2f x = new MatOfPoint2f();
                x.fromArray(contour.toArray());
                RotatedRect rect = Imgproc.minAreaRect(x);
                if (rect.center.x < CROP) continue;
                double size = Math.hypot(rect.size.height, rect.size.width);
                if (size > max) {
                    max = size;
                    best = rect;
                }
            }
            if (best == null) {
                location = GoldLocation.RIGHT;
                return;
            }
            x = best.center.x;
            y = best.center.y;
            if (y > THRESHOLD) location = GoldLocation.LEFT;
            else location = GoldLocation.CENTER;
            Imgproc.rectangle(in, new Point(best.boundingRect().x, best.boundingRect().y), new Point(best.boundingRect().x + best.boundingRect().width, best.boundingRect().y + best.boundingRect().height), new Scalar(255, 0, 0), 5);
        } else location = GoldLocation.RIGHT;
    }

    private static boolean display() {
        i++;
        return i == DISPLAY;
    }

    public GoldLocation getLocation() {
        return location;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

}
