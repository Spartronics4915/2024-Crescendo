// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2024;

import com.spartronics4915.frc2024.subsystems.Intake;
import com.spartronics4915.frc2024.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static com.spartronics4915.frc2024.Constants.OI.kDriverControllerPort;
import static com.spartronics4915.frc2024.Constants.OI.kOperatorControllerPort;
import static com.spartronics4915.frc2024.Constants.OI.kDriverTriggerDeadband;
import static com.spartronics4915.frc2024.Constants.OI.kOperatorTriggerDeadband;

public class RobotContainer {
    private static final CommandXboxController mDriverController = new CommandXboxController(kDriverControllerPort);
    private static final CommandXboxController mOperatorController = new CommandXboxController(kOperatorControllerPort);
    
    private static final Intake mIntake = Intake.getInstance();
    
    public RobotContainer() {
        ShuffleboardTab overviewTab = Shuffleboard.getTab("Overview");
        configureBindings();
    }

    private void configureBindings() {
        mOperatorController.leftTrigger(kOperatorTriggerDeadband)
            .onTrue(mIntake.setStateCommand(IntakeState.IN))
            .onFalse(mIntake.setStateCommand(IntakeState.OFF));

        mOperatorController.rightTrigger(kOperatorTriggerDeadband)
            .onTrue(mIntake.setStateCommand(IntakeState.OUT))
            .onFalse(mIntake.setStateCommand(IntakeState.OFF));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
