package com.acmerobotics.roverruckus.robot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.drive.Kinematics;
import com.acmerobotics.roadrunner.drive.MecanumKinematics;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roverruckus.hardware.LynxOptimizedI2cFactory;
import com.acmerobotics.roverruckus.trajectory.Trajectory;
import com.acmerobotics.roverruckus.util.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@Config
public class MecanumDrive extends Subsystem{

    public static double P = 10;
    public static double I = 4;
    public static double D = 0;
    public static double F = 12.579;
    public static double TELEOP_V = 45;
    public static double TELEOP_OMEGA = 5;

    private DcMotorEx[] motors;
    private static final String[] motorNames = {
            "m0",
            "m1",
            "m2",
            "m3"
    };
    private static final int[] hubs = {
            0, 0, 0, 0
    };
    private static final Vector2d[] wheelPositions = {
            new Vector2d(5.5,7.5),
            new Vector2d(-5.5, 7.5),
            new Vector2d(-5.5,-7.5),
            new Vector2d(5.5, -7.5)
    };
    private static final Vector2d[] rotorDirections = {
            new Vector2d(1,-1),
            new Vector2d(1, 1),
            new Vector2d(1, -1),
            new Vector2d(1, 1)
    };
    private static final double radius = 2;
//    private static final double ticksPerInch = (2240 * 4) / (radius * 2 * Math.PI);

    public static double axialMaxV = 10;
    public static double axialMaxA = 10;
    public static double axialMaxJ = 10;

    public static double headingMaxV = 5;
    public static double headingMaxA = 5;
    public static double headingMaxJ = 5;

    public static double HOLD_POSITION_P = 1;
    public static double HOLD_POSITION_I = 1;
    public static double HOLD_POSITION_HEADING_P = 1;
    public static double HOLD_POSITION_HEADING_I = 1;

    private PIDController holdPositionController;
    private PIDController holdPositionHeadingController;

    private Pose2d targetVelocity = new Pose2d(0,0,0);
    private Pose2d currentEstimatedPose = new Pose2d(0, 0, 0);
    private boolean estimatingPose = true;
    private double[] lastWheelPositions = new double[4];
    private long lastUpdate = 0;
    private double lastHeading = 0;

    public Trajectory trajectory;
    private long startTime;
    private Robot robot;

    private BNO055IMU imu;

    private enum Mode {
        OPEN_LOOP,
        FOLLOWING_PATH,
        HOLD_POSITION
    }

    private Mode currentMode = Mode.OPEN_LOOP;

    public MecanumDrive(Robot robot, HardwareMap hardwareMap) {
        this.robot = robot;
        motors = new DcMotorEx[motorNames.length];
        for (int i = 0; i < motorNames.length; i++) {
            motors[i] = (DcMotorEx) hardwareMap.get(DcMotor.class, motorNames[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            robot.addMotor(motors[i]);
        }
        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);

        I2cDeviceSynch imuI2cDevice = LynxOptimizedI2cFactory.createLynxI2cDeviceSynch(hardwareMap.get(LynxModule.class, "hub1"), 0);
        imuI2cDevice.setUserConfiguredName("imu");
        imu = new LynxEmbeddedIMU(imuI2cDevice);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        try {
            // axis remap
            byte AXIS_MAP_CONFIG_BYTE = 0b00011000; //swaps y-z, 0b00100001 is y-x, 0x6 is x-z
            byte AXIS_MAP_SIGN_BYTE = 0b001; //x, y, z

            //Need to be in CONFIG mode to write to registers
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

            Thread.sleep(100); //Changing modes requires a delay before doing anything else

            //Write to the AXIS_MAP_CONFIG register
            imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);

            //Write to the AXIS_MAP_SIGN register
            imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);

            //Need to change back into the IMU mode to use the gyro
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);

            Thread.sleep(100); //Changing modes again requires a delay
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        setMotorPIDF(P, I, D, F);

    }

    private void internalSetVelocity (Pose2d v) {
        for (int i = 0; i < motors.length; i++) {
            Vector2d rotorVelocity = new Vector2d(
              v.getX() - v.getHeading() * wheelPositions[i].getY(),
              v.getY() + v.getHeading() * wheelPositions[i].getX()
            );
            double surfaceVelocity = rotorVelocity.dot(rotorDirections[i]);
            double wheelVelocity = surfaceVelocity / radius;
            motors[i].setVelocity(wheelVelocity, AngleUnit.RADIANS);
        }
    }

    /**
     * Set the robot velocity for open-loop control
     *
     * @param target Desired velocity of the robot, on [-1, 1], will be scaled to max V
     */
    public void setVelocity (Pose2d target) {
        if (currentMode == Mode.HOLD_POSITION && target.getY() == 0 && target.getX() == 0 && target.getHeading() == 0) return;
        double v = target.pos().norm();
        v = Range.clip(v, -1, 1) * TELEOP_V;
        double theta = Math.atan2(target.getX(), target.getY());
        double omega = target.getHeading() * TELEOP_OMEGA;

        targetVelocity = new Pose2d(v * Math.cos(theta), v * Math.sin(theta), omega);

        currentMode = Mode.OPEN_LOOP;
    }

    public void setRealVelocity(Pose2d target) {
        this.targetVelocity = target;
    }

    public void followPath (Path path) {
        trajectory = new Trajectory(path);
        currentMode = Mode.FOLLOWING_PATH;
        startTime = System.currentTimeMillis();
    }

    public boolean isFollowingPath () {
        if (currentMode != Mode.FOLLOWING_PATH) return false;
        return !trajectory.isComplete();
    }

    public Pose2d getCurrentEstimatedPose() {
        return currentEstimatedPose;
    }

    public void stop() {
        targetVelocity = new Pose2d(0,0,0);
        currentMode = Mode.OPEN_LOOP;
    }

    @Override
    public void update(TelemetryPacket packet) {
        packet.put("mode", currentMode);
        if (estimatingPose) {
            updatePoseEstimate();
            drawPose(packet.fieldOverlay(), currentEstimatedPose, "red");
        }

        switch (currentMode) {
            case OPEN_LOOP:
                packet.put("vX", targetVelocity.getX());
                packet.put("vY", targetVelocity.getY());
                packet.put("vOmega", targetVelocity.getHeading());
                internalSetVelocity(targetVelocity);
                break;

            case FOLLOWING_PATH:
                internalSetVelocity(Kinematics.fieldToRobotPoseVelocity(currentEstimatedPose, trajectory.update((System.currentTimeMillis() - startTime) / 1000.0, currentEstimatedPose, packet)));
                if (trajectory.isComplete()) currentMode = Mode.OPEN_LOOP;
                break;

            case HOLD_POSITION:
                Vector2d trackingError = currentEstimatedPose.pos();
                double headingError = currentEstimatedPose.getHeading();
                double correctionMag = holdPositionController.update(trackingError.norm());
                Vector2d correction = trackingError.div(trackingError.norm()).times(correctionMag);
                double headingCorrection = holdPositionHeadingController.update(headingError);
                internalSetVelocity(new Pose2d(correction, headingCorrection));
        }

    }

    private void updatePoseEstimate() {
        if (lastUpdate == 0) {
            lastUpdate = System.currentTimeMillis();
            lastHeading = imu.getAngularOrientation().firstAngle;
            for (int i = 0; i < 4; i++) {
                lastWheelPositions[i] = motors[i].getCurrentPosition();
            }
            return;
        }

        List<Double> wheelVelocities = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            double pos = robot.getEncoderPosition(motors[i]);
            double distance = ((pos - lastWheelPositions[i])/motors[i].getMotorType().getTicksPerRev()) * (Math.PI * 4);
            wheelVelocities.add(distance);
            lastWheelPositions[i] = pos;
        }
        Pose2d v = MecanumKinematics.wheelToRobotVelocities(wheelVelocities, 16, 16);
        double currentHeadding = imu.getAngularOrientation().firstAngle;
        v = new Pose2d(v.pos(), currentHeadding - lastHeading);
        lastHeading = currentHeadding;
        currentEstimatedPose = Kinematics.relativeOdometryUpdate(currentEstimatedPose, v);
    }

    public void setMotorPIDF(double p, double i, double d, double f) {
        for (DcMotorEx motor: motors) motor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void setMotorPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
        for (DcMotorEx motor: motors) motor.setPIDFCoefficients(mode, coefficients);
    }

    public PIDFCoefficients getMotorPIDF() {
        return motors[0].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void drawPose (Canvas overlay, Pose2d pose, String color) {
        overlay.setStroke(color);
        double sin = Math.sin(pose.getHeading()) * 9;
        double cos = Math.cos(pose.getHeading()) * 9;
        overlay.strokeCircle(pose.getX(), pose.getY(), 9);
        overlay.strokeLine(pose.getX(), pose.getY(), pose.getX() + cos, pose.getY() + sin);
    }

    @Override
    public boolean isBusy() {
        return isFollowingPath();
    }

    public void setCurrentEstimatedPose(Pose2d pose) {
        lastHeading = imu.getAngularOrientation().firstAngle;
        currentEstimatedPose = pose;
    }

    public double getVelocity() {
        double v = 0;
        for (DcMotorEx motor: motors) {
            v += motor.getVelocity(AngleUnit.RADIANS) * 2;
        }
        v /= motors.length;
        return v;
    }

    public void holdPosition () {
        holdPositionController = new PIDController(HOLD_POSITION_P, HOLD_POSITION_I, 0);
        holdPositionHeadingController = new PIDController(HOLD_POSITION_HEADING_P, HOLD_POSITION_HEADING_I, 0);
        setCurrentEstimatedPose(new Pose2d(0,0,0));
        currentMode = Mode.HOLD_POSITION;
        estimatingPose = true;
    }
}
