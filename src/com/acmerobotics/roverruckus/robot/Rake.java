package com.acmerobotics.roverruckus.robot;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.util.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * Created by Emma Sheffo on 11/16/2018.
 */

public class Rake extends Subsystem{

    public static double ARM_PULLEY_RADIUS = 0.6835; // in
    public static double MAX_EXTENSION_DISTANCE = 41;
    public static PIDCoefficients ARM_PID = new PIDCoefficients(-3, 0, -0.04);

    public enum ArmMode {
        PID,
        MANUAL
    }


    private DcMotor rakeArm;

    private int encoderOffset;

    private PIDController armController;
    private double armPower;

    private ArmMode armMode = ArmMode.MANUAL;

    private TelemetryData telemetryData;

    private class TelemetryData {
        public ArmMode rakeArmMode;
        public double rakeArmPower;
        public double rakeArmPosition;
        public double rakeArmError;
    }

    public Rake(HardwareMap map) {
        telemetryData = new TelemetryData();

        rakeArm = new CachingDcMotorEx(map.get(DcMotorEx.class,"rakeArm"));
        rakeArm.setDirection(DcMotorSimple.Direction.REVERSE);

        armController = new PIDController(ARM_PID);

    }

    public void setArmPower(double power) {
        armPower = power;
        armMode = ArmMode.MANUAL;
    }

    public void setArmPosition(double distance) {
        armController.setSetpoint(distance);
        armController.reset();
        armMode = ArmMode.PID;
    }

    public void resetEncoder() {
        encoderOffset = rakeArm.getCurrentPosition();
    }

    public int getEncoderPosition() {
        return rakeArm.getCurrentPosition() - encoderOffset;
    }

    public double getArmPosition() {
        int encoderPosition = getEncoderPosition();
        double revs = encoderPosition / rakeArm.getMotorType().getTicksPerRev();
        return 2 * Math.PI * ARM_PULLEY_RADIUS * revs;
    }

    public ArmMode getArmMode() {
        return armMode;
    }

    @Override
    public void update(TelemetryPacket packet) {

        switch (armMode) {
            case MANUAL:
                break;
            case PID:
                double armPosition = getArmPosition();
                double armError = armController.getError(armPosition);
                armPower = armController.update(armError);

                telemetryData.rakeArmPosition = armPosition;
                telemetryData.rakeArmError = armError;

                break;
        }

        rakeArm.setPower(armPower);

        telemetryData.rakeArmMode = armMode;
        telemetryData.rakeArmPower = armPower;


    }


}


