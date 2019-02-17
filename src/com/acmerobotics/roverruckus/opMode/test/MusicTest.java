package com.acmerobotics.roverruckus.opMode.test;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;

@Autonomous(name="musicTest")
public class MusicTest extends LinearOpMode {

    @Override public void runOpMode () {

//        SoundPool pool = new SoundPool.Builder().build();
//        int id = pool.load("/sdcard/tokyo_drift.mp33", 1);
        MediaPlayer media = MediaPlayer.create(hardwareMap.appContext, R.raw.tokyo_drift);


        waitForStart();
        media.start();

//        id = pool.play(id, 1,1,1,0,1);

        while (!isStopRequested()) ;
        media.stop();
        media.release();

//        pool.stop(id);
    }
}
