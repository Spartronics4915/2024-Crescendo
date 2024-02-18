// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2024;

import com.spartronics4915.frc2024.util.ModeSwitchInterface;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command mAutonomousCommand;

    private RobotContainer mRobotContainer;

    @Override
    public void robotInit() {
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
        System.out.println("STARTING AUTO INIT");
        modeInit();
        System.out.println("MODE INIT DONE");
        mAutonomousCommand = mRobotContainer.getAutonomousCommand();
        System.out.println("GOT AUTO COMMAND");
        if (mAutonomousCommand != null) {
            System.out.println("AUTO IS NOT NULL (GOOD)");
            mAutonomousCommand.schedule();
            System.out.println("AUTO SCHEDULED");
        } else System.out.println("AUTO IS NULL (BAD)");
        System.out.println("AUTO INIT DONE");
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        System.out.println("STARTING TELEOP INIT");
        modeInit();
        System.out.println("MODE INIT DONE");
        if (mAutonomousCommand != null) {
            System.out.println("AUTO IS NOT NULL (BAD)");
            mAutonomousCommand.cancel();
            System.out.println("AUTO CANCELLED");
        } else System.out.println("AUTO IS NULL (GOOD)");
        System.out.println("TELEOP INIT DONE");
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
            trapezoid.modeSwitchAction();
        }
    }
}
