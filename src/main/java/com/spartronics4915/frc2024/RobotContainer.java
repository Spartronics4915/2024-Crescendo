// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2024;

import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.subsystems.Bling;
import com.spartronics4915.frc2024.subsystems.Elevator;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.spartronics4915.frc2024.commands.LockOnCommand;
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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static com.spartronics4915.frc2024.Constants.Drive.kPPConfig;
import static com.spartronics4915.frc2024.Constants.OI.kDriverControllerPort;
import static com.spartronics4915.frc2024.Constants.OI.kOperatorControllerPort;
import static com.spartronics4915.frc2024.Constants.OI.kDriverTriggerDeadband;
import static com.spartronics4915.frc2024.Constants.OI.kOperatorTriggerDeadband;
import static com.spartronics4915.frc2024.util.ModeSwitchInterface.ModeSwitchSubsystems;

import java.util.ArrayList;

public class RobotContainer {
    private static final CommandXboxController mDriverController = new CommandXboxController(kDriverControllerPort);
    private static final CommandXboxController mOperatorController = new CommandXboxController(kOperatorControllerPort);

    // private static final Intake mIntake = Intake.getInstance();
    // private static final IntakeWrist mIntakeWrist = IntakeWrist.getInstance();
    private static final SwerveDrive mSwerveDrive = SwerveDrive.getInstance();

    // private static final ShooterWrist mShooter = ShooterWrist.getInstance();
    // private static final Elevator mElevator = Elevator.getInstance();
    // private static final Bling mBling = Bling.getInstance();

    // private static final TrapezoidSimulator mSimulator;
    private final SwerveSim mSwerveSim;

    // static {
    //     ModeSwitchSubsystems.add(mIntakeWrist);
    //     ModeSwitchSubsystems.add(mShooter);
    //     ModeSwitchSubsystems.add(mElevator);

    //     ArrayList<TrapezoidSimulatorInterface> list = new ArrayList<>();
    //     list.add(mIntakeWrist);
    //     list.add(mShooter);
    //     list.add(mElevator);
    //     mSimulator = new TrapezoidSimulator(list);

    //     ModeSwitchSubsystems.add(mElevator);
    // }

    public RobotContainer() {
        ShuffleboardTab overviewTab = Shuffleboard.getTab("Overview");
        mSwerveSim = new SwerveSim(mSwerveDrive);
        configureBindings();

        VisionSubsystem.getInstance(); // ensures VisionSubsystem is created so the limelights log (for debug)
    }

    public static CommandXboxController getDriverController() {
        return mDriverController;
    }

    public static CommandXboxController getOperatorController() {
        return mOperatorController;
    }

    private void configureBindings() {
        mDriverController.a().onTrue(mSwerveDrive.toggleFieldRelativeCommand());

        // mOperatorController.leftTrigger(kOperatorTriggerDeadband) // TODO: change
        //         .onTrue(mIntake.setStateCommand(IntakeState.IN))
        //         .onFalse(mIntake.setStateCommand(IntakeState.OFF));

        // mOperatorController.rightTrigger(kOperatorTriggerDeadband) // TODO: change
        //         .onTrue(mIntake.setStateCommand(IntakeState.OUT))
        //         .onFalse(mIntake.setStateCommand(IntakeState.OFF));

        // mDriverController.leftTrigger(kDriverTriggerDeadband)
        //         .whileTrue(new LockOnCommand());
        // 
        // mOperatorController.povUp().whileTrue(mShooter.manualRunCommand(Rotation2d.fromDegrees(1)));
        // mOperatorController.povDown().whileTrue(mShooter.manualRunCommand(Rotation2d.fromDegrees(-1)));

        // mOperatorController.povRight().whileTrue(mIntakeWrist.manualRunCommand(Rotation2d.fromDegrees(1)));
        // mOperatorController.povLeft().whileTrue(mIntakeWrist.manualRunCommand(Rotation2d.fromDegrees(-1)));

        // mOperatorController.rightBumper().whileTrue(mElevator.manualRunCommand(Rotation2d.fromDegrees(1)));
        // mOperatorController.leftBumper().whileTrue(mElevator.manualRunCommand(Rotation2d.fromDegrees(-1)));

        // mOperatorController.a().onTrue(mIntakeWrist.setStateCommand(IntakeAssemblyState.GROUNDPICKUP));
        // mOperatorController.y().onTrue(mIntakeWrist.setStateCommand(IntakeAssemblyState.SOURCE));
        // mOperatorController.x().onTrue(mIntakeWrist.setStateCommand(IntakeAssemblyState.AMP));
        // mOperatorController.b().onTrue(mIntakeWrist.setStateCommand(IntakeAssemblyState.STOW)); //TEMP        
    }

    public Command getAutonomousCommand() {
        // Pose2d newPose = new Pose2d(new Translation2d(3, 3), new Rotation2d());
        // Command initPose = Commands.runOnce(() -> mSwerveDrive.resetPose(newPose));
        // Command driveBackwards = new DriveStraightCommands.DriveStraightFixedDistance(mSwerveDrive, new Rotation2d(),
        //         2, new TrapezoidProfile.Constraints(0.5, 0.5 / 2));
        // Command holdStill = Commands.run(() -> mSwerveDrive.drive(
        //         new ChassisSpeeds(0, 0, 0), false));
        // return Commands.sequence(initPose, driveBackwards, holdStill);
        
        // return new PathPlannerAuto("Test Auto");

        // return new PathPlannerAuto("Path 1 Only");
        
        return new PathPlannerAuto("Auto 2");
    }
}
