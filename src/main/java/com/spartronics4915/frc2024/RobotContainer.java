// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2024;

import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.Constants.BlingModes;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.commands.IntakeAssemblyCommands;
import com.spartronics4915.frc2024.subsystems.Bling;
import com.spartronics4915.frc2024.subsystems.Elevator;
import com.spartronics4915.frc2024.subsystems.Shooter;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.spartronics4915.frc2024.commands.BootCoralCommand;
import com.spartronics4915.frc2024.commands.LockOnCommand;
import com.spartronics4915.frc2024.commands.MovingAutoAimCommand;
import com.spartronics4915.frc2024.commands.ToggleDetectorCommand;
import com.spartronics4915.frc2024.commands.drivecommands.DriveStraightCommands;
import com.spartronics4915.frc2024.commands.drivecommands.DriveStraightCommands.DriveStraightFixedDistance;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator;
import com.spartronics4915.frc2024.subsystems.Bling.BlingMCwithPriority;
import com.spartronics4915.frc2024.subsystems.Bling.BlingMC;
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
import com.spartronics4915.frc2024.util.NoteVisualizer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.spartronics4915.frc2024.Constants.AutoAimConstants.kAutoAimTarget;
import static com.spartronics4915.frc2024.Constants.AutoAimConstants.kStageTarget;
import static com.spartronics4915.frc2024.Constants.Drive.kPPConfig;
import static com.spartronics4915.frc2024.Constants.OI.kDriverControllerPort;
import static com.spartronics4915.frc2024.Constants.OI.kOperatorControllerPort;
import static com.spartronics4915.frc2024.Constants.OI.kDriverTriggerDeadband;
import static com.spartronics4915.frc2024.Constants.OI.kOperatorTriggerDeadband;
import static com.spartronics4915.frc2024.util.ModeSwitchInterface.ModeSwitchSubsystems;

import java.util.ArrayList;
import java.util.Optional;
import java.util.Set;

public class RobotContainer {
    // private enum SubsystemFlags{
    //     IntakeWristFlag (true),
    //     IntakeFlag (false),
    //     ShooterFlag (false),
    //     ShooterWristFlag (true),
    //     ElevatorFlag (true);

    //     private final boolean isUsed;
    //     private SubsystemFlags(boolean isUsed) {this.isUsed = isUsed;}
    // }

    private static final CommandXboxController mDriverController = new CommandXboxController(kDriverControllerPort);
    private static final CommandXboxController mOperatorController = new CommandXboxController(kOperatorControllerPort);

    private final SendableChooser<Command> mAutoChooser;

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

    private final Bling mBling = Bling.getInstance();

    private static final PowerDistribution mPDP = new PowerDistribution();

    static {

        ArrayList<TrapezoidSimulatorInterface> list = new ArrayList<>();
        
        mIntakeWrist = IntakeWrist.getInstance();;
        ModeSwitchSubsystems.add(mIntakeWrist);
        list.add(mIntakeWrist);
        

        mIntake = Intake.getInstance();
        ModeSwitchSubsystems.add(mIntake);
        

        mShooterWrist = ShooterWrist.getInstance();
        ModeSwitchSubsystems.add(mShooterWrist);
        list.add(mShooterWrist);

        mShooter = Shooter.getInstance();
        ModeSwitchSubsystems.add(mShooter);

        mElevator = Elevator.getInstance();
        ModeSwitchSubsystems.add(mElevator);
        list.add(mElevator);
        

        mSimulator = new TrapezoidSimulator(list);

        ModeSwitchSubsystems.add(mElevator);
        
        // Bling.addToLinkedList(new BlingMCwithPriority(() -> {
        //     if (mPDP.getStickyFaults().Brownout) {
        //         return Optional.of(new BlingMC(BlingModes.PULSE_SWITCH, Color.kRed, Color.kDarkRed));
        //     } else {
        //         return Optional.empty();
        //     }
        // }, -1));

        new Trigger(() -> {return mPDP.getFaults().Brownout;}).onTrue(Commands.runOnce(() -> {
            DriverStation.reportError("BROWNOUT DETECTED", false);
        }));
    }

    public RobotContainer() {
        mAutoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", mAutoChooser);

        ShuffleboardTab overviewTab = Shuffleboard.getTab(ShuffleBoard.UserTab);
        overviewTab.add(mAutoChooser);
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
        mOperatorController.povUp().whileTrue(mShooterWrist.manualRunCommand(Rotation2d.fromDegrees(1)));
        mOperatorController.povDown().whileTrue(mShooterWrist.manualRunCommand(Rotation2d.fromDegrees(-1)));
        mOperatorController.povRight().whileTrue(mIntakeWrist.manualRunCommand(Rotation2d.fromDegrees(1)));
        mOperatorController.povLeft().whileTrue(mIntakeWrist.manualRunCommand(Rotation2d.fromDegrees(-1)));

        mOperatorController.rightBumper().whileTrue(mElevator.manualRunCommand(Rotation2d.fromDegrees(2.5)));
        mOperatorController.leftBumper().whileTrue(mElevator.manualRunCommand(Rotation2d.fromDegrees(-2.5)));

        var commandFactory = new IntakeAssemblyCommands(mIntakeWrist, mIntake, mElevator); 
        mOperatorController.a().onTrue(commandFactory.setState(IntakeAssemblyState.GROUNDPICKUP));
        mOperatorController.y().onTrue(commandFactory.setState(IntakeAssemblyState.SOURCE));
        mOperatorController.x().onTrue(commandFactory.setState(IntakeAssemblyState.AMP));
        mOperatorController.b().onTrue(commandFactory.setState(IntakeAssemblyState.STOW)); //TEMP
    
        mOperatorController.a().onTrue(mShooter.setShooterStateCommand(ShooterState.OFF));
        mOperatorController.y().onTrue(mShooter.setShooterStateCommand(ShooterState.ON));

        mOperatorController.x().onTrue(mShooter.setConveyorStateCommand(ConveyorState.OFF));
        mOperatorController.b().onTrue(mShooter.setConveyorStateCommand(ConveyorState.IN));

        mOperatorController.button(10).whileTrue(NoteVisualizer.visualizeTrajectoryCommand());

        mOperatorController.button(13)
                .whileTrue(new MovingAutoAimCommand(com.spartronics4915.frc2024.Constants.AutoAimConstants.kAutoAimTarget));

        mDriverController.a()
                .whileTrue(new ToggleDetectorCommand());
        
    mOperatorController.button(15).onTrue(mSwerveDrive.toggleFieldRelativeCommand());

        mDriverController.leftTrigger(kDriverTriggerDeadband)
                .whileTrue(new LockOnCommand());
    }

    public Command getAutonomousCommand() {
        return mAutoChooser.getSelected();
    }
}
