// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2024;

import com.spartronics4915.frc2024.util.ModeSwitchInterface;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;

import edu.wpi.first.wpilibj.DataLogManager;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public class Robot extends TimedRobot {
    private Command mAutonomousCommand;

    private RobotContainer mRobotContainer;
    public static DataLog log;
    public static final Timer TELEOP_TIMER = new Timer();

    @Override
    public void robotInit() {

        DataLogManager.logNetworkTables(true);
        
        log = DataLogManager.getLog();
        
        DriverStation.startDataLog(log, true);

        mRobotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        modeInit();
        mAutonomousCommand = mRobotContainer.getAutonomousCommand();
        if (mAutonomousCommand != null) {
            mAutonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        modeInit();
        if (mAutonomousCommand != null) {
            mAutonomousCommand.cancel();
        }
        TELEOP_TIMER.start();

        if(RobotBase.isSimulation()) {
            
            mRobotContainer.getSwerveDrive().resetPose(new Pose2d(2,7, new Rotation2d()));
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        modeInit();
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    /**
     * this method is called every time a robot is enabled or disabled
     */
    public void modeInit(){
        
        //Safety for trapezoid
        for (var trapezoid : ModeSwitchInterface.ModeSwitchSubsystems) {
            if (trapezoid != null) trapezoid.modeSwitchAction();
        }
    }
}
