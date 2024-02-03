// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2024;

import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake.IntakeState;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.TrapezoidSimulatorInterface;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static com.spartronics4915.frc2024.Constants.OI.kDriverControllerPort;
import static com.spartronics4915.frc2024.Constants.OI.kOperatorControllerPort;
import static com.spartronics4915.frc2024.Constants.OI.kDriverTriggerDeadband;
import static com.spartronics4915.frc2024.Constants.OI.kOperatorTriggerDeadband;
import static com.spartronics4915.frc2024.util.TrapezoidSubsystemInterface.TrapezoidSubsystems;

import java.util.ArrayList;

public class RobotContainer {
    private static final CommandXboxController mDriverController = new CommandXboxController(kDriverControllerPort);
    private static final CommandXboxController mOperatorController = new CommandXboxController(kOperatorControllerPort);
    
    // private static final Intake mIntake = Intake.getInstance();
    private static final IntakeWrist mIntakeWrist = IntakeWrist.getInstance();

    private static final ShooterWrist mShooter = ShooterWrist.getInstance();

    private static final TrapezoidSimulator mSimulator;

    static{
        TrapezoidSubsystems.add(mIntakeWrist);
        TrapezoidSubsystems.add(mShooter);
        ArrayList<TrapezoidSimulatorInterface> list = new ArrayList<>();
        list.add(mIntakeWrist);
        list.add(mShooter);
        mSimulator = new TrapezoidSimulator(list);
    }
    
    public RobotContainer() {
        ShuffleboardTab overviewTab = Shuffleboard.getTab("Overview");
        configureBindings();
        // VisionSubsystem.getInstance(); //ensures VisionSubsystem is created so the limelights log (for debug)
    }

    private void configureBindings() {
        // mOperatorController.leftTrigger(kOperatorTriggerDeadband)
        //     .onTrue(mIntake.setStateCommand(IntakeState.IN))
        //     .onFalse(mIntake.setStateCommand(IntakeState.OFF));

        // mOperatorController.rightTrigger(kOperatorTriggerDeadband)
        //     .onTrue(mIntake.setStateCommand(IntakeState.OUT))
        //     .onFalse(mIntake.setStateCommand(IntakeState.OFF));


        //TODO switch to a Command file for the intakeAssembly commands 

        mOperatorController.povUp().whileTrue(mShooter.manualRunCommand(Rotation2d.fromDegrees(1)));
        mOperatorController.povDown().whileTrue(mShooter.manualRunCommand(Rotation2d.fromDegrees(-1)));

        mOperatorController.povRight().whileTrue(mIntakeWrist.manualRunCommand(Rotation2d.fromDegrees(1)));
        mOperatorController.povLeft().whileTrue(mIntakeWrist.manualRunCommand(Rotation2d.fromDegrees(-1)));


        mOperatorController.a().onTrue(mIntakeWrist.setStateCommand(IntakeAssemblyState.GROUNDPICKUP));
        mOperatorController.y().onTrue(mIntakeWrist.setStateCommand(IntakeAssemblyState.SOURCE));
        mOperatorController.x().onTrue(mIntakeWrist.setStateCommand(IntakeAssemblyState.AMP));
        mOperatorController.b().onTrue(mIntakeWrist.setStateCommand(IntakeAssemblyState.STOW)); //TEMP


    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
