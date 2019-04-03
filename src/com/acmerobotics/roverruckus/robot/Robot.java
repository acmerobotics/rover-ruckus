package com.acmerobotics.roverruckus.robot;

import android.app.Activity;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.hardware.CachingAnalogInput;
import com.acmerobotics.roverruckus.hardware.CachingDcMotorEx;
import com.acmerobotics.roverruckus.hardware.CachingMotor;
import com.acmerobotics.roverruckus.hardware.CachingServo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.configuration.OpModeConfiguration;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CountDownLatch;

public class Robot implements OpModeManagerNotifier.Notifications, GlobalWarningSource {

    public static final String TAG = "RobotLog";

    public MecanumDrive drive;
    public Lift lift;
    public Intake intake;
    private List<Subsystem> subsystems;

    private final Map<DcMotorController, LynxModuleIntf> hubs;
    private final Map<LynxModuleIntf, LynxGetBulkInputDataResponse> responses;
    private LynxModuleIntf[] hubsByIndex = new LynxModuleIntf[2];
    private boolean updated = false;

    private OpModeManagerImpl opModeManager;

    private Thread subsystemExecutor, telemetryExecutor;

    private BlockingQueue<TelemetryPacket> packets;

    private Map<String, Object> telemetry;
    private List<String> telemetryLines;

    private LinearOpMode master;
    private HardwareMap map;

    public OpModeConfiguration config;

    private List<CachingMotor> motors;

    public Robot(LinearOpMode opMode, HardwareMap map) {
        this.master = opMode;
        this.map = map;
        subsystems = new ArrayList<>();
        packets = new EvictingBlockingQueue<>(new ArrayBlockingQueue<TelemetryPacket>(10));
        telemetry = new HashMap<>();
        telemetryLines = new ArrayList<>();
        config = new OpModeConfiguration(map.appContext);
        this.motors = new ArrayList<>();

        try {
        drive = new MecanumDrive(this, map);
        subsystems.add(drive);
        } catch (Exception e) {
            telemetryLines.add("problem with drive");
        }
        try {
        intake = new Intake(this, map);
        subsystems.add(intake);
        } catch (Exception e) {
            telemetryLines.add("problem with intake");

        }
        try {
        lift = new Lift(this, map);
        subsystems.add(lift);
        } catch (Exception e) {
            telemetryLines.add("problem with lift");
        }

        hubs = new HashMap<>(2);
        responses = new HashMap<>(2);
        try {
            hubs.put(
                    map.get(DcMotorController.class, "hub1"),
                    map.get(LynxModuleIntf.class, "hub1"));
            hubsByIndex[0] = map.get(LynxModuleIntf.class, "hub1");
        } catch (Exception e) {
            telemetryLines.add("problem with hub1");
        }

        try {
            hubs.put(
                    map.get(DcMotorController.class, "hub2"),
                    map.get(LynxModuleIntf.class, "hub2"));
            hubsByIndex[1] = map.get(LynxModuleIntf.class, "hub2");
        } catch (Exception e) {
            telemetryLines.add("problem with hub2");
        }


        RobotLog.registerGlobalWarningSource(this);
        Activity activity = (Activity) opMode.hardwareMap.appContext;
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

//        subsystemExecutor = new Thread(new Runnable() {
//            @Override
//            public void run() {
//                while (!Thread.currentThread().isInterrupted()) {
//                    updateSubsystems();
//                }
//            }
//        });
//        subsystemExecutor.start();
////
//        telemetryExecutor = new Thread(new Runnable() {
//            @Override
//            public void run() {
//                while (!Thread.currentThread().isInterrupted()) {
//                    updateTelemetry();
//                }
//            }
//        });
//        telemetryExecutor.start();

    }

    public void updateSubsystems() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.addTimestamp();
        for (Subsystem subsystem : subsystems) {
            subsystem.update(packet);
        }
        synchronized (telemetry) {
            packet.putAll(telemetry);
        }
        synchronized (telemetryLines) {
            for (String line : telemetryLines) packet.addLine(line);
        }
        packets.offer(packet);
    }

    public void updateHardware () {
        for (CachingMotor motor: motors) {
            motor.update();
        }

        for (LynxModuleIntf hub : hubs.values()) {
            LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(hub);
            try {
                responses.put(hub, command.sendReceive());
                updated = true;
            } catch (Exception e) {
                try {
                    Log.e(TAG,  "get bulk data error");
                    Log.e(TAG, e.getLocalizedMessage());
                } catch (NullPointerException npe) {
                    Log.e(TAG, "this is really funny");
                }
                updated = false;
            }
        }

        telemetry.put("hubs updated", updated);
        if (updated) Log.i(TAG, "hardware updated");
    }

    public int getEncoderPosition(DcMotor motor) {
        if (!updated) return 0;
        synchronized (responses) {
            return responses.get(hubs.get(motor.getController())).getEncoder(motor.getPortNumber()) * (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1);
        }
    }

    public boolean getDigitalPort(int hub, int port) {
        if (!updated) return false;
        synchronized (responses) {
            return responses.get(hubsByIndex[hub]).getDigitalInput(port);
        }
    }

    public int getAnalogPort(int hub, int port) {
        if (!updated) return 0;
        synchronized (responses) {
            return responses.get(hubsByIndex[hub]).getAnalogInput(port);
        }
    }

    public int getMotorVelocity(DcMotor motor) {
        if (!updated) return 0;
        synchronized (responses) {
            return responses.get(hubs.get(motor.getController())).getVelocity(motor.getPortNumber()) * (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1);
        }
    }

    public void addTelemetry(String key, Object value) {
        synchronized (telemetry) {
            telemetry.put(key, value);
        }
    }

    public void addBulkTelemetry(Map<String, Object> map) {
        telemetry.putAll(map);
    }

    public void addLine(String line) {
        synchronized (telemetryLines) {
            telemetryLines.add(line);
        }
    }

    public void clearTelemetry() {
        synchronized (telemetry) {
            telemetry.clear();
        }
        synchronized (telemetryLines) {
            telemetryLines.clear();
        }
    }


    public void updateTelemetry() {
        try {
            FtcDashboard.getInstance().sendTelemetryPacket(packets.remove());

        } catch (NoSuchElementException nse) {

        }
    }

    public void stop() {
//        subsystemExecutor.interrupt();
//        telemetryExecutor.interrupt();
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
        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
            opModeManager = null;
        }
    }

    @Override
    public String getGlobalWarning() {
        return "";
    }

    @Override
    public void suppressGlobalWarning(boolean suppress) {

    }

    @Override
    public void setGlobalWarning(String warning) {

    }

    @Override
    public void clearGlobalWarning() {
    }

    public void waitForAllSubsystems() {
        for (; ; ) {
            update();
            boolean complete = true;
            telemetryLines.clear();
            for (Subsystem subsystem : subsystems) {
                if (subsystem.isBusy()) {
                    String string = subsystem.getClass().getCanonicalName() + " is busy";
                    if (!telemetryLines.contains(string))
                        telemetryLines.add(string);
                    complete = false;
                }
            }
            if (complete || master.isStopRequested()) return;
        }
    }

    public void update() {
        long start = System.currentTimeMillis();
        updateSubsystems();
        long subsystemEnd = System.currentTimeMillis();
        updateTelemetry();
        long telemetryEnd = System.currentTimeMillis();
        updateHardware();
        long end = System.currentTimeMillis();
        telemetry.put("looptime", end - start);
        telemetry.put("subsystemTime", subsystemEnd - start);
        telemetry.put("telemetryTime", telemetryEnd - subsystemEnd);
        telemetry.put("hardwareTime", end - telemetryEnd);
    }


    public void pause(long millis) {
        long start = System.currentTimeMillis();
        while (((millis + start) - System.currentTimeMillis()) > 0) {
            update();
        }
    }

    public CachingDcMotorEx getMotor (String deviceName) {
        CachingDcMotorEx motor = new CachingDcMotorEx(this, map.get(DcMotorEx.class, deviceName));
        this.motors.add(motor);
        return motor;
    }

    public CachingAnalogInput getAnalogInput (int hub, int port) {
        return new CachingAnalogInput(this, hub, port);
    }

    public CachingServo getServo (String deviceName) {
        CachingServo servo = new CachingServo(map.get(Servo.class, deviceName));
        motors.add(servo);
        return servo;
    }
}
