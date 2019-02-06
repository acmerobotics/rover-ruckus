package com.acmerobotics.roverruckus.robot;

import android.content.Context;
import android.content.SharedPreferences;

public class RobotState {

    public static final String PREFS_NAME = "robot_state";
    public static final String LIFT_OFFSET = "lift_offset";
    public static final String RAKE_OFFSET = "rake_offset";

    private SharedPreferences prefs;

    public RobotState(Context context) {
        prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
    }

    public void setLiftOffset(double position) {
        SharedPreferences.Editor editor = prefs.edit();
        editor.putFloat(LIFT_OFFSET, (float) position);
        editor.commit();
    }

    public double getLiftOffset() {
        return prefs.getFloat(LIFT_OFFSET, 0);
    }

    public void setRakeOffset(double position) {
        SharedPreferences.Editor editor = prefs.edit();
        editor.putFloat(RAKE_OFFSET, (float) position);
        editor.commit();
    }

    public double getRakeOffset() {
        return prefs.getFloat(RAKE_OFFSET, 0);
    }

}
