// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2024;

import com.spartronics4915.frc2024.commands.drivecommands.DriveStraightCommands;
import com.spartronics4915.frc2024.commands.drivecommands.DriveStraightCommands.DriveStraightFixedDistance;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake.IntakeState;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.TrapezoidSimulatorInterface;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveSim;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    private static final Intake mIntake = Intake.getInstance();
    private static final IntakeWrist mIntakeWrist = IntakeWrist.getInstance();
    private static final SwerveDrive mSwerveDrive = SwerveDrive.getInstance();

    private static final TrapezoidSimulator mSimulator;
    private final SwerveSim mSwerveSim;

    static {
        TrapezoidSubsystems.add(mIntakeWrist);
        ArrayList<TrapezoidSimulatorInterface> list = new ArrayList<>();
        list.add(mIntakeWrist);
        mSimulator = new TrapezoidSimulator(list);
    }

    public RobotContainer() {
        ShuffleboardTab overviewTab = Shuffleboard.getTab("Overview");
        mSwerveSim = new SwerveSim(mSwerveDrive);
        configureBindings();
        VisionSubsystem.getInstance(); // ensures VisionSubsystem is created so the limelights log (for debug)
    }

    public CommandXboxController getDriverController() {
        return mDriverController;
    }

    public CommandXboxController getOperatorController() {
        return mOperatorController;
    }

    private void configureBindings() {
        mOperatorController.leftTrigger(kOperatorTriggerDeadband) // TODO: change
                .onTrue(mIntake.setStateCommand(IntakeState.IN))
                .onFalse(mIntake.setStateCommand(IntakeState.OFF));

        mOperatorController.rightTrigger(kOperatorTriggerDeadband) // TODO: change
                .onTrue(mIntake.setStateCommand(IntakeState.OUT))
                .onFalse(mIntake.setStateCommand(IntakeState.OFF));
    }

    public Command getAutonomousCommand() {

        Pose2d newPose = new Pose2d(new Translation2d(3, 3), new Rotation2d());
        Command initPose = Commands.runOnce(() -> mSwerveDrive.resetPose(newPose));
        Command driveBackwards = new DriveStraightCommands.DriveStraightFixedDistance(mSwerveDrive, new Rotation2d(),
                2, new TrapezoidProfile.Constraints(0.5, 0.5 / 2));
        Command holdStill = Commands.run(() -> mSwerveDrive.drive(
                new ChassisSpeeds(0, 0, 0), false));
        return Commands.sequence(initPose, driveBackwards, holdStill);
    }
}
