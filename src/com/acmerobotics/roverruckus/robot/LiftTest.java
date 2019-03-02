package com.acmerobotics.roverruckus.robot;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;

public class LiftTest {
    public static void main (String[] args) {
        System.out.println(Lift.V);
        double k = (18.0 / 25.0);
        System.out.println(Lift.V / k);
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0,0,0,0),
                new MotionState(Lift.LIFT_SCORE, 0,0,0),
                Lift.V / k,
                Lift.A,
                Lift.J
        );
        System.out.println (profile.duration());
    }
}
