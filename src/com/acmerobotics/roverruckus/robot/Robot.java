package com.acmerobotics.roverruckus.robot;

import android.app.Activity;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.Executor;

public class Robot implements OpModeManagerNotifier.Notifications, GlobalWarningSource {

    public MecanumDrive drive;
    private final List<Subsystem> subsystems;
    private final List<Subsystem> subsystemsWithProblem;

    private List<CachingMotor> motors;

    private OpModeManagerImpl opModeManager;

    private Thread subsystemExecutor, motorExecutor, telemetryExecutor;

    private BlockingQueue<TelemetryPacket> packets;

    private Map<String, Object> telemetry;
    private List<String> telemetryLines;

    private OpMode master;

    public Robot(OpMode opMode, HardwareMap map) {
        this.master = opMode;
        motors = new ArrayList<>();
        subsystems = new ArrayList<>();
        subsystemsWithProblem = new ArrayList<>();
        packets = new EvictingBlockingQueue<>(new ArrayBlockingQueue<TelemetryPacket>(10));
        telemetry = new HashMap<>();
        telemetryLines = new ArrayList<>();

        try {
            drive = new MecanumDrive(this, map);
            subsystems.add(drive);
        } catch (Exception e) {
            synchronized(subsystemsWithProblem) {
                if (!subsystemsWithProblem.contains(drive)) subsystemsWithProblem.add(drive);
            }
        }

        RobotLog.registerGlobalWarningSource(this);
        Activity activity = (Activity) opMode.hardwareMap.appContext;
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        subsystemExecutor = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.currentThread().isInterrupted()) {
                    updateSubsystems();
                }
            }
        });
        subsystemExecutor.start();

        telemetryExecutor = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.currentThread().isInterrupted()) {
                    updateTelemetry();
                }
            }
        });
        telemetryExecutor.start();
//        motorExecutor.execute(new Runnable() {
//            @Override
//            public void run() {
//                while (!Thread.currentThread().isInterrupted()) {
//                    updateMotors();
//                }
//            }
//        });


    }

    private void updateMotors() {
        for (CachingMotor motor: motors) {
            motor.update();
        }
    }

    public void updateSubsystems() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.addTimestamp();
        for (Subsystem subsystem: subsystems) {
            try  {
                subsystem.update(packet);

                synchronized (subsystemsWithProblem) {
                    subsystemsWithProblem.remove(subsystem);
                }
            } catch (Exception e) {
                synchronized (subsystemsWithProblem) {
                    if (!subsystemsWithProblem.contains(subsystem)) subsystemsWithProblem.add(drive);
                }
            }
        }
        synchronized (telemetry) {
            packet.putAll(telemetry);
        }
        synchronized (telemetryLines) {
            for (String line : telemetryLines) packet.addLine(line);
        }
        packets.offer(packet);
        updateMotors();
    }


    public void addTelemetry(String key, Object value) {
        synchronized (telemetry) {
            telemetry.put(key, value);
        }
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
            FtcDashboard.getInstance().sendTelemetryPacket(packets.take());

        } catch (InterruptedException ie) {

        }
    }

    public void stop() {
        subsystemExecutor.interrupt();
        telemetryExecutor.interrupt();
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
        StringBuilder builder = new StringBuilder();
        for (Subsystem subsystem: subsystemsWithProblem) {
            builder.append(subsystem.getClass().getSimpleName());
            builder.append('\n');
        }
        return builder.toString();
    }

    @Override
    public void suppressGlobalWarning(boolean suppress) {

    }

    @Override
    public void setGlobalWarning(String warning) {

    }

    @Override
    public void clearGlobalWarning() {
        synchronized (subsystemsWithProblem) {
            subsystemsWithProblem.clear();
        }
    }
}
