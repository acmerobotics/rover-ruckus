package com.acmerobotics.roverruckus.trajectory;

import com.acmerobotics.roadrunner.Vector2d;

public class TrackingWheelKinematics {

    private Vector2d r0, r1;

    public TrackingWheelKinematics (double a0, double a1) {
        if (a0 == a1) throw new IllegalArgumentException("you can't point them in the same direction silly!!!");
        double a = Math.cos(a0);
        double b = Math.sin(a0);
        double c = Math.cos(a1);
        double d = Math.sin(a1);
        double determinant = a*d - b*c;
        r0 = new Vector2d(d, -b).div(determinant);
        r1 = new Vector2d(-c, a).div(determinant);
        System.out.println(r0);
        System.out.println(r1);
    }

    public Vector2d transform (Vector2d dist) {
        return new Vector2d(r0.dot(dist), r1.dot(dist));
    }

}
