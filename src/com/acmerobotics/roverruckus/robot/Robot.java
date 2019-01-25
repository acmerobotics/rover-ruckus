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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

public class Robot implements OpModeManagerNotifier.Notifications, GlobalWarningSource {

    public MecanumDrive drive;
    public Lift lift;
    public Intake intake;
    public Placer placer;
    private List<Subsystem> subsystems;

    private Map<DcMotorController, LynxModuleIntf> hubs;
    private Map<LynxModuleIntf, LynxGetBulkInputDataResponse> responses;
    private boolean updated = false;

    private OpModeManagerImpl opModeManager;

    private Thread subsystemExecutor, telemetryExecutor;

    private BlockingQueue<TelemetryPacket> packets;

    private Map<String, Object> telemetry;
    private List<String> telemetryLines;

    private LinearOpMode master;

    public Robot(LinearOpMode opMode, HardwareMap map) {
        this.master = opMode;
        subsystems = new ArrayList<>();
        packets = new EvictingBlockingQueue<>(new ArrayBlockingQueue<TelemetryPacket>(10));
        telemetry = new HashMap<>();
        telemetryLines = new ArrayList<>();

//        try {
            drive = new MecanumDrive(this, map);
            subsystems.add(drive);
//        } catch (Exception e) {
//            telemetryLines.add("problem with drive");
//        }
//        try {
            intake = new Intake(this, map);
            subsystems.add(intake);
//        } catch (Exception e) {
//            telemetryLines.add("problem with intake");
//
//        }
//        try {
            lift = new Lift(this, map);
            subsystems.add(lift);
//        } catch (Exception e) {
//            telemetryLines.add("problem with lift");
//        }
//        try {
            placer = new Placer(map);
            subsystems.add(placer);
//        } catch (Exception e) {
//            telemetryLines.add("problem with placer");
//        }

        hubs = new HashMap<>(2);
        responses = new HashMap<>(2);
        try {
            hubs.put(
                    map.get(DcMotorController.class, "hub1"),
                    map.get(LynxModuleIntf.class, "hub1"));
        } catch (Exception e) {
            telemetryLines.add("problem with hub1");
        }

        try {
            hubs.put(
                    map.get(DcMotorController.class, "hub2"),
                    map.get(LynxModuleIntf.class, "hub2"));
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
        for (LynxModuleIntf hub: hubs.values()) {
            LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(hub);
            try {
                responses.put(hub, command.sendReceive());
                updated = true;
            } catch (Exception e) {
                Log.e("get bulk data error", e.getMessage());
                updated = false;
            }
        }

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

    public int getEncoderPosition(DcMotor motor) {
        if (!updated) return 0;
        synchronized (responses) {
            return responses.get(hubs.get(motor.getController())).getEncoder(motor.getPortNumber()) * (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1);
        }
    }

    public boolean getDigitalPort(LynxModuleIntf hub, int port) {
        if (!updated) return false;
        synchronized (responses) {
            return responses.get(hub).getDigitalInput(port);
        }
    }

    public int getAnalogPort(LynxModuleIntf hub, int port) {
        if (!updated) return 0;
        synchronized (responses) {
            return responses.get(hub).getAnalogInput(port);
        }
    }

    public int getMotorVelocity(DcMotor motor) {
        if (!updated) return 0;
        synchronized (responses) {
            return responses.get(hubs.get(motor.getController())).getVelocity(motor.getPortNumber());
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
                if (subsystem.isBusy()) {
                    telemetryLines.add(subsystem.getClass().getSimpleName() + " is busy");
                    complete = false;
                }
            }
            if (complete || master.isStopRequested()) return;
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
