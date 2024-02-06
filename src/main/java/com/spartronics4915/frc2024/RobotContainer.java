// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2024;

import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.commands.IntakeAssemblyCommands;
import com.spartronics4915.frc2024.subsystems.Elevator;
import com.spartronics4915.frc2024.subsystems.Shooter;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.spartronics4915.frc2024.commands.LockOnCommand;
import com.spartronics4915.frc2024.commands.ToggleDetectorCommand;
import com.spartronics4915.frc2024.commands.drivecommands.DriveStraightCommands;
import com.spartronics4915.frc2024.commands.drivecommands.DriveStraightCommands.DriveStraightFixedDistance;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake.IntakeState;
import com.spartronics4915.frc2024.subsystems.Shooter.ConveyorState;
import com.spartronics4915.frc2024.subsystems.Shooter.ShooterState;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.TrapezoidSimulatorInterface;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveSim;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;
import com.spartronics4915.frc2024.util.ModeSwitchInterface;

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
    private enum SubsystemFlags{
        IntakeWristFlag (true),
        IntakeFlag (false),
        ShooterFlag (false),
        ShooterWristFlag (true),
        ElevatorFlag (true);

        private final boolean isUsed;
        private SubsystemFlags(boolean isUsed) {this.isUsed = isUsed;}
    }

    private static final CommandXboxController mDriverController = new CommandXboxController(kDriverControllerPort);
    private static final CommandXboxController mOperatorController = new CommandXboxController(kOperatorControllerPort);
    
    // private static final Intake mIntake = Intake.getInstance();
    
    private static final IntakeWrist mIntakeWrist;
    private static final Intake mIntake;
    private static final ShooterWrist mShooterWrist;
    private static final Shooter mShooter;
    private static final Elevator mElevator;

    private static final SwerveDrive mSwerveDrive = SwerveDrive.getInstance();
    
    private static final TrapezoidSimulator mSimulator;
    private final SwerveSim mSwerveSim;

    private final VisionSubsystem mVision;

    static {

        ArrayList<TrapezoidSimulatorInterface> list = new ArrayList<>();
        
        if (SubsystemFlags.IntakeWristFlag.isUsed){
            mIntakeWrist = IntakeWrist.getInstance();;
            ModeSwitchSubsystems.add(mIntakeWrist);
            list.add(mIntakeWrist);
        } else mIntakeWrist = null;

        if (SubsystemFlags.IntakeFlag.isUsed){
            mIntake = Intake.getInstance();
            ModeSwitchSubsystems.add(mIntake);
        } else mIntake = null;

        if (SubsystemFlags.ShooterWristFlag.isUsed){
            mShooterWrist = ShooterWrist.getInstance();
            ModeSwitchSubsystems.add(mShooterWrist);
            list.add(mShooterWrist);
        } else mShooterWrist = null;

        if (SubsystemFlags.ShooterFlag.isUsed){
            mShooter = Shooter.getInstance();
        } else mShooter = null;

        if (SubsystemFlags.ElevatorFlag.isUsed){
            mElevator = Elevator.getInstance();
            ModeSwitchSubsystems.add(mElevator);
            list.add(mElevator);
        } else mElevator = null;

        mSimulator = new TrapezoidSimulator(list);

        ModeSwitchSubsystems.add(mElevator);
    }

    public RobotContainer() {
        ShuffleboardTab overviewTab = Shuffleboard.getTab("Overview");
        mSwerveSim = new SwerveSim(mSwerveDrive);
        mVision = VisionSubsystem.getInstance();
        configureBindings();
    }

    public static CommandXboxController getDriverController() {
        return mDriverController;
    }

    public static CommandXboxController getOperatorController() {
        return mOperatorController;
    }

    private void configureBindings() {
        if (SubsystemFlags.ShooterWristFlag.isUsed) {
            mOperatorController.povUp().whileTrue(mShooterWrist.manualRunCommand(Rotation2d.fromDegrees(1)));
            mOperatorController.povDown().whileTrue(mShooterWrist.manualRunCommand(Rotation2d.fromDegrees(-1)));
        }
        if (SubsystemFlags.IntakeWristFlag.isUsed) {
            mOperatorController.povRight().whileTrue(mIntakeWrist.manualRunCommand(Rotation2d.fromDegrees(1)));
            mOperatorController.povLeft().whileTrue(mIntakeWrist.manualRunCommand(Rotation2d.fromDegrees(-1)));
        }

        if (SubsystemFlags.ElevatorFlag.isUsed) {
            mOperatorController.rightBumper().whileTrue(mElevator.manualRunCommand(Rotation2d.fromDegrees(1)));
            mOperatorController.leftBumper().whileTrue(mElevator.manualRunCommand(Rotation2d.fromDegrees(-1)));
        }

        if (SubsystemFlags.IntakeWristFlag.isUsed && SubsystemFlags.ElevatorFlag.isUsed) {
            var commandFactory = new IntakeAssemblyCommands(mIntakeWrist, mIntake, mElevator); 
            mOperatorController.a().onTrue(commandFactory.setState(IntakeAssemblyState.GROUNDPICKUP));
            mOperatorController.y().onTrue(commandFactory.setState(IntakeAssemblyState.SOURCE));
            mOperatorController.x().onTrue(commandFactory.setState(IntakeAssemblyState.AMP));
            mOperatorController.b().onTrue(commandFactory.setState(IntakeAssemblyState.STOW)); //TEMP
        }
        
        if (SubsystemFlags.ShooterFlag.isUsed) {
            mOperatorController.a().onTrue(mShooter.setShooterStateCommand(ShooterState.OFF));
            mOperatorController.y().onTrue(mShooter.setShooterStateCommand(ShooterState.ON));

            mOperatorController.x().onTrue(mShooter.setConveyorStateCommand(ConveyorState.OFF));
            mOperatorController.b().onTrue(mShooter.setConveyorStateCommand(ConveyorState.IN));
        }

        mDriverController.a().onTrue(mSwerveDrive.toggleFieldRelativeCommand());

        mDriverController.leftTrigger(kDriverTriggerDeadband)
                .whileTrue(new LockOnCommand());

        mDriverController.a()
                .whileTrue(new ToggleDetectorCommand());
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
