package com.acmerobotics.roverruckus.trajectory;

import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.util.Angle;

public class GoodLinearInterpolator extends HeadingInterpolator {

    private double startHeading, endHeading, turnAngle;

    public GoodLinearInterpolator (double startHeading, double endHeading) {
        this.endHeading = endHeading;
        this.startHeading = startHeading;
        turnAngle = endHeading - startHeading;
        if (turnAngle > Math.PI ) turnAngle -= Math.PI;
    }

    @Override
    public boolean respectsDerivativeContinuity() {
        return false;
    }

    @Override
    public double get(double v) {
        return Angle.norm(startHeading + v / parametricCurve.length() * turnAngle);
    }

    @Override
    public double deriv(double v) {
        return turnAngle / parametricCurve.length();
    }

    @Override
    public double secondDeriv(double v) {
        return 0;
    }
}
