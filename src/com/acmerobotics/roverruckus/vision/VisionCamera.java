package com.acmerobotics.roverruckus.vision;

import android.app.Activity;
import android.util.Log;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCamera2View;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;

public class VisionCamera implements OpModeManagerNotifier.Notifications, JavaCamera2View.CvCameraViewListener2 {

    public static final String TAG = "VisionCamera";

    private Activity activity;
    private List<Tracker> trackers;
    private JavaCamera2View view;

    public VisionCamera () {
        activity = AppUtil.getInstance().getActivity();
        OpModeManagerImpl.getOpModeManagerOfActivity(activity).registerListener(this);
        trackers = new ArrayList<>();

        final CountDownLatch countDownLatch = new CountDownLatch(1);

        final BaseLoaderCallback baseLoaderCallback = new BaseLoaderCallback(activity) {
            @Override
            public void onManagerConnected(int status) {
                if (status == LoaderCallbackInterface.SUCCESS) {
                    Log.i(TAG, "opencv load successful");
                    countDownLatch.countDown();
                } else {
                    Log.e(TAG, "error loading opencv");
                }

            }
        };

        AppUtil.getInstance().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_4_0, activity, baseLoaderCallback);
            }
        });

        try {
            countDownLatch.await();
        } catch (InterruptedException e) {
            Log.e(TAG, "interrupted while loading opencv");
        }

        view = (JavaCamera2View) activity.findViewById(com.qualcomm.ftcrobotcontroller.R.id.cameraViewId);
        view.setCvCameraViewListener(this);
        view.setCam
        view.setMaxFrameSize(4000, 4000);
        AppUtil.getInstance().runOnUiThread(() -> {
            view.enableView();
            view.setVisibility(View.VISIBLE);
        });

    }

    private void stop() {
        Log.i(TAG, "stopped");
        AppUtil.getInstance().runOnUiThread(() -> {
            view.disableView();
            view.setVisibility(View.INVISIBLE);
        });
    }

    public void addTracker (Tracker tracker) {
        trackers.add(tracker);
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        stop();
    }

    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat frame = inputFrame.rgba();
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2RGB);
        Log.i(TAG, "" + frame.type());
        for (Tracker tracker: trackers) {
            tracker.processFrame(frame);
        }
        return frame;
    }
}
