package com.acmerobotics.roverruckus.robot;

import android.app.Activity;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roverruckus.hardware.CachingMotor;
import com.acmerobotics.roverruckus.hardware.CachingSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxDekaInterfaceCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelCurrentAlertLevelCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelCurrentAlertLevelResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

public class Robot implements OpModeManagerNotifier.Notifications, GlobalWarningSource {

    public MecanumDrive drive;
    public Lift lift;
    public Intake intake;
    private final List<Subsystem> subsystems;

    private List<CachingMotor> motors;
    private List<CachingSensor> sensors;
    private LynxModuleIntf[] hubs = new LynxModuleIntf[2];
    private List<LynxGetBulkInputDataResponse> responses;
    private boolean updated = false;

    private OpModeManagerImpl opModeManager;

    private Thread subsystemExecutor, hardwareExecutor, telemetryExecutor;

    private BlockingQueue<TelemetryPacket> packets;

    private Map<String, Object> telemetry;
    private List<String> telemetryLines;

    private OpMode master;

    public Robot(OpMode opMode, HardwareMap map) {
        this.master = opMode;
        motors = new ArrayList<>();
        sensors = new ArrayList<>();
        subsystems = new ArrayList<>();
        packets = new EvictingBlockingQueue<>(new ArrayBlockingQueue<TelemetryPacket>(10));
        telemetry = new HashMap<>();
        telemetryLines = new ArrayList<>();

//        try {
            drive = new MecanumDrive(this, map);
            subsystems.add(drive);
//        } catch (Exception e) {
//
//        }
        try {
            intake = new Intake(this, map);
            subsystems.add(intake);
        } catch (Exception e) {

        }
        try {
            lift = new Lift(this, map);
            subsystems.add(lift);
        } catch (Exception e) {

        }

        hubs[0] = map.get(LynxModuleIntf.class, "hub1");
        //hubs[1] = map.get(LynxModuleIntf.class, "hub2");

        responses = new ArrayList<>(2);

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
//
//        telemetryExecutor = new Thread(new Runnable() {
//            @Override
//            public void run() {
//                while (!Thread.currentThread().isInterrupted()) {
//                    updateTelemetry();
//                }
//            }
//        });
//        telemetryExecutor.start();
//
//        hardwareExecutor = new Thread(new Runnable() {
//            @Override
//            public void run() {
//                while (!Thread.currentThread().isInterrupted()) {
//                    updateHardware();
//                }
//            }
//        });


    }

    private void updateHardware() {
        for (CachingMotor motor: motors) {
            motor.update();
        }
        for (CachingSensor sensor: sensors) {
            sensor.update();
        }
        updated = true;
        synchronized (responses) {
            for (int i = 0; i < hubs.length; i++) {
                try {
                    LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(hubs[i]);
                    responses.set(i, command.sendReceive());
                } catch (Exception e) {
                    Log.e("robot", "error in updating bulk input data");
                    updated = false;
                }
            }
        }
    }

    public void updateSubsystems() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.addTimestamp();
        for (Subsystem subsystem: subsystems) {
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

    public int getEncoderPosition(int hub, int port) {
        if (!updated) return 0;
        synchronized (responses) {
            return responses.get(hub).getEncoder(port);
        }
    }

    public boolean getDigitalPort(int hub, int port) {
        synchronized (responses) {
            return responses.get(hub).getDigitalInput(port);
        }
    }

    public int getAnalogPort(int hub, int port) {
        synchronized (responses) {
            return responses.get(hub).getAnalogInput(port);
        }
    }

    public int getMotorVelocity(int hub, int port) {
        synchronized (responses) {
            return responses.get(0).getVelocity(port);
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


    protected void addMotor(CachingMotor motor) {
        motors.add(motor);
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
//        StringBuilder builder = new StringBuilder();
//        for (Subsystem subsystem: subsystemsWithProblem) {
//            builder.append(subsystem.getClass().getName());
//            builder.append('\n');
//        }
//        return builder.toString();
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
//        synchronized (subsystemsWithProblem) {
//            subsystemsWithProblem.clear();
//        }
    }

    public void waitForAllSubsystems() {
        for (;;) {
            update();
            boolean complete = true;
            for (Subsystem subsystem : subsystems) {
                if (subsystem.isBusy()) complete = false;
            }
            if (complete) return;
        }
    }

    public void update() {
        updateSubsystems();
        updateTelemetry();
//        updateHardware();
    }


//    public void pause (long millis) {
//        long start = System.currentTimeMillis();
//
// long remaining;
//        while ((remaining = (millis - (System.currentTimeMillis() - start)) > 0) {
//            try {
//                Thread.sleep(end - System.currentTimeMillis());
//            }
//        }
//    }
}
