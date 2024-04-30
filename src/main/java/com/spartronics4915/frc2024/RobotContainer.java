// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2024;

import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.Constants.BlingModes;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.Constants.ShooterWristConstants.ShooterWristState;
import com.spartronics4915.frc2024.commands.IntakeAssemblyCommands;
import com.spartronics4915.frc2024.commands.LimelightAuto;
import com.spartronics4915.frc2024.subsystems.Bling;
import com.spartronics4915.frc2024.subsystems.Elevator;
import com.spartronics4915.frc2024.subsystems.Shooter;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.spartronics4915.frc2024.commands.AlignToSpeakerCommand;
import com.spartronics4915.frc2024.commands.AutoComponents;
import com.spartronics4915.frc2024.commands.AutoFactory;
import com.spartronics4915.frc2024.commands.DigestCommands;
import com.spartronics4915.frc2024.commands.LockOnCommand;
import com.spartronics4915.frc2024.commands.MovingAutoAimCommand;
import com.spartronics4915.frc2024.commands.StationaryAutoAimCommand;
import com.spartronics4915.frc2024.commands.StationaryAutoAimVisionPose;
import com.spartronics4915.frc2024.commands.TableAutoAimCommand;
import com.spartronics4915.frc2024.commands.AutoFactory.PathSet;
import com.spartronics4915.frc2024.commands.AutoFactory.StartingPosition;
import com.spartronics4915.frc2024.commands.advancedAutos.AdvAutoLogic;
import com.spartronics4915.frc2024.commands.visionauto.ShooterRunFireControl;
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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
    // IntakeWristFlag (true),
    // IntakeFlag (false),
    // ShooterFlag (false),
    // ShooterWristFlag (true),
    // ElevatorFlag (true);

    // private final boolean isUsed;
    // private SubsystemFlags(boolean isUsed) {this.isUsed = isUsed;}
    // }

    private static final CommandXboxController mDriverController = new CommandXboxController(kDriverControllerPort);
    private static final CommandXboxController mOperatorController = new CommandXboxController(
            kOperatorControllerPort);

    private final SendableChooser<Command> mAutoChooser;

    // private static final Intake mIntake = Intake.getInstance();

    private static final IntakeWrist mIntakeWrist;
    private static final Intake mIntake;

    private static final ShooterWrist mShooterWrist;
    private static final Shooter mShooter;
    private static final Elevator mElevator;

    private final SwerveDrive mSwerveDrive;

    private static final TrapezoidSimulator mSimulator;
    private final SwerveSim mSwerveSim;

    private static final VisionSubsystem mVision = VisionSubsystem.getInstance();
    private static final ShooterRunFireControl shooterFireControl = new ShooterRunFireControl(
            mVision.getSpeakerTagLocator());

    private final Bling mBling;

    private static final PowerDistribution mPDP = new PowerDistribution();

    public static boolean addTad = false;

    public static final double britishTad = 0.1;

    static {

        ArrayList<TrapezoidSimulatorInterface> list = new ArrayList<>();

        mIntakeWrist = IntakeWrist.getInstance();
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

        // ModeSwitchSubsystems.add(mElevator);

        // Bling.addToLinkedList(new BlingMCwithPriority(() -> {
        // if (mPDP.getStickyFaults().Brownout) {
        // return Optional.of(new BlingMC(BlingModes.PULSE_SWITCH, Color.kRed, Color.kDarkRed));
        // } else {
        // return Optional.empty();
        // }
        // }, -1));

        // new Trigger(() -> {
        // return mPDP.getFaults().Brownout;
        // }).onTrue(Commands.runOnce(() -> {
        // DriverStation.reportError("BROWNOUT DETECTED", false);
        // }));

        var mIntake = Intake.getInstance();
        ShuffleboardTab tab = Shuffleboard.getTab(ShuffleBoard.DebugTab);
        tab.add("ground intake", AutoComponents.groundIntake());
        tab.add("Load into shooter", AutoComponents.loadIntoShooter());
        tab.add("Shoot from loaded", AutoComponents.shootFromLoaded());
        tab.add("Stationary Aim And Shoot", AutoComponents.stationaryAimAndShoot());
        tab.add("intake out", mIntake.setStateCommand(IntakeState.OUT));
        tab.add("intake off", mIntake.setStateCommand(IntakeState.OFF));
        tab.add("intake in", mIntake.setStateCommand(IntakeState.IN));
        tab.add("ScanAimAuto", AdvAutoLogic.visionAimAndShoot());
        tab.add("SearchGather", AdvAutoLogic.searchAndGather());
        tab.add("Spin", AdvAutoLogic.searchForNote().withTimeout(2));
        tab.add("MAA 3 seconds",  Commands.defer(() -> {
            final var alliance = DriverStation.getAlliance().get();
            final var speaker = alliance == Alliance.Blue
                    ? AutoComponents.BLUE_SPEAKER
                    : AutoComponents.RED_SPEAKER;
            return new MovingAutoAimCommand(speaker);
        }, Set.of()).withTimeout(3));
    }

    public RobotContainer() {
        mSwerveDrive = SwerveDrive.getInstance();
        mSwerveSim = new SwerveSim(mSwerveDrive);
        if (mSwerveDrive != null) {
            // NamedCommands.registerCommand("intake", AutoComponents.groundToIntake());
            NamedCommands.registerCommand("load", AutoComponents.loadIntoShooter());
            NamedCommands.registerCommand("aim",
                    Commands.defer(() -> new StationaryAutoAimCommand(
                            AutoComponents.getTarget().get()), Set.of()));
            NamedCommands.registerCommand("shoot", AutoComponents.shootFromLoaded());
            NamedCommands.registerCommand("shootPreloaded", AutoComponents.shootPreloaded());
            NamedCommands.registerCommand("shootFromLoaded", AutoComponents.shootFromLoaded());
            NamedCommands.registerCommand("shooterOn", mShooter.setShooterStateCommand(ShooterState.ON));
            NamedCommands.registerCommand("stationaryAutoAim",
                    AutoComponents.stationaryAutoAim().withTimeout(2)); // TODO:
                                                                        // replace
                                                                        // timeout
                                                                        // with
                                                                        // debounced
                                                                        // atTarget
            NamedCommands.registerCommand("aimAndShootPreloaded", AutoComponents.stationaryAimAndShoot());
            NamedCommands.registerCommand("groundIntake", AutoComponents.groundIntake());
            NamedCommands.registerCommand("loadIntoShooter", AutoComponents.loadIntoShooter());
            NamedCommands.registerCommand("DriveToPickUpNote", LimelightAuto.driveToNote());
            NamedCommands.registerCommand("StopChassis", Commands.runOnce(() -> {
                mSwerveDrive.drive(new ChassisSpeeds(0, 0, 0), false);
            }));
            NamedCommands.registerCommand("InitShooterFireControl", shooterFireControl.initRunCommand());
            NamedCommands.registerCommand("ShootNote1", shooterFireControl.aimAndFireCommand(20));
            NamedCommands.registerCommand("ShootNote2", shooterFireControl.aimAndFireCommand(20));
            NamedCommands.registerCommand("ShootNote3", shooterFireControl.aimAndFireCommand(20));

            // NamedCommands for Full Composite Autos
            NamedCommands.registerCommand("CenterFourNote", mShooter.setShooterStateCommand(ShooterState.ON)
                    .andThen(CompositeAutos.generateCenterFourNote()));
            NamedCommands.registerCommand("FastCenterFourNote",
                    mShooter.setShooterStateCommand(ShooterState.ON)
                            .andThen(CompositeAutos.generateCenterFourNoteFaster()));
            NamedCommands.registerCommand("MiddleLower2", mShooter.setShooterStateCommand(ShooterState.ON)
                    .andThen(CompositeAutos.lowerTwoNote()));

            // mAutoChooser = AutoBuilder.buildAutoChooser();
            mAutoChooser = buildAutoChooser();

            SmartDashboard.putData("Auto Chooser", mAutoChooser);

            SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

            ShuffleboardTab overviewTab = Shuffleboard.getTab(ShuffleBoard.UserTab);
            overviewTab.add(mAutoChooser);

            Shuffleboard.getTab("Tab 12").add(mAutoChooser);

            SmartDashboard.putData(CommandScheduler.getInstance());
        } else {
            mAutoChooser = null;
        }
        mBling = Bling.getInstance();
        mBling.setMode(BlingModes.OFF);
        configureBindings();
    }

    public static ShooterRunFireControl getShooterFireControl() {
        return shooterFireControl;
    }

    public static CommandXboxController getDriverController() {
        return mDriverController;
    }

    public static CommandXboxController getOperatorController() {
        return mOperatorController;
    }

    private void configureBindings() { // TODO: format these nicely

        // driver controls

        if (mSwerveDrive != null) {
            mDriverController.a().onTrue(mSwerveDrive.toggleFieldRelativeCommand());
            mDriverController.b().onTrue(mSwerveDrive.resetYawCommand());
        }

        // mDriverController.leftStick().onTrue(new HomingCommand());
        mDriverController.leftStick().onTrue(IntakeAssemblyCommands.stow());

        // mDriverController.rightBumper().onTrue(LimelightAuto.driveToNote());
        mDriverController.rightBumper().toggleOnTrue(LimelightAuto.followNote());

        mDriverController.leftTrigger(kDriverTriggerDeadband)
                .whileTrue(new LockOnCommand(mVision.getNoteLocator()));
        mDriverController.x().onTrue(new AlignToSpeakerCommand().withTimeout(1.25));

        mDriverController.leftBumper().onTrue(new AlignToSpeakerCommand().withTimeout(1.25));

        mDriverController.y().onTrue(mIntakeWrist.resetEncoderToAngle(-31)); // ground intake minus one

        mDriverController.x()
                .onTrue(mShooterWrist.resetToAngle(
                        ShooterWristState.HARD_STOP.shooterAngle.getDegrees() + 1));


        Shuffleboard.getTab("Tab 12").addBoolean("AddTad", () -> {
            return addTad;
        });
        mDriverController.back().toggleOnTrue(Commands.startEnd(
            () -> {
                addTad = true;

            }, () ->{
                addTad = false;
            }
        ));

        // Operator controls
        // Buttons:
        mOperatorController.x().onTrue(IntakeAssemblyCommands.ComplexSetState(IntakeAssemblyState.AMP));
        mOperatorController.y().onTrue(
            AutoComponents.loadIntoShooter()
            .andThen(mShooter.setShooterStateCommand(ShooterState.ON))
        );
        mOperatorController.a()
                .onTrue(IntakeAssemblyCommands.ComplexSetState(IntakeAssemblyState.GROUNDPICKUP));
        mOperatorController.b().onTrue(Commands.parallel(
                mIntake.setStateCommand(IntakeState.OFF),
                mShooter.setShooterStateCommand(ShooterState.OFF),
                mShooter.setConveyorStateCommand(ConveyorState.OFF)));

        // manual controls

        mOperatorController.rightBumper().whileTrue(mElevator.manualRunCommand(0.02));
        mOperatorController.leftBumper().whileTrue(mElevator.manualRunCommand(-0.02));

        mOperatorController.povUp().whileTrue(mShooterWrist.manualRunCommand(Rotation2d.fromDegrees(0.45)));
        mOperatorController.povDown().whileTrue(mShooterWrist.manualRunCommand(Rotation2d.fromDegrees(-0.45)));

        mOperatorController.povRight().whileTrue(mIntakeWrist.manualRunCommand(Rotation2d.fromDegrees(0.45)));
        mOperatorController.povLeft().whileTrue(mIntakeWrist.manualRunCommand(Rotation2d.fromDegrees(-0.45)));

        // misc
        mOperatorController.leftStick()
                .onTrue(mShooterWrist.setStateCommand(ShooterWristState.SUB_TELE));

        mOperatorController.rightStick()
                .onTrue(mElevator.setTargetCommand(IntakeAssemblyState.Climb));

        mOperatorController.back() // menu
                .onTrue(Commands.parallel(
                        mElevator.setTargetCommand(IntakeAssemblyState.STOW),
                        mIntakeWrist.setStateCommand(IntakeAssemblyState.STOW),
                        mShooterWrist.setStateCommand(ShooterWristState.STOW)));

        mOperatorController.start()
                .whileTrue(DigestCommands.out());

        // triggers

        // mOperatorController.leftTrigger(kOperatorTriggerDeadband).whileTrue(
        //         Commands.repeatingSequence(
        //                 Commands.defer(() -> {
        //                     final var alliance = DriverStation.getAlliance().get();
        //                     final var speaker = alliance == Alliance.Blue
        //                             ? AutoComponents.BLUE_SPEAKER
        //                             : AutoComponents.RED_SPEAKER;
        //                     return Commands.parallel(
        //                             new TableAutoAimCommand(),
        //                             // new StationaryAutoAimCommand(speaker)
        //                             StationaryAutoAimVisionPose
        //                                     .getStationaryAutoAimVisionOrPose(
        //                                             mVision.getSpeakerTagLocator(),
        //                                             speaker));
        //                 }, Set.of())));
        mOperatorController.leftTrigger(kDriverTriggerDeadband)
                .whileTrue(new LockOnCommand(mVision.getNoteLocator()));

        // mDriverController.povUp().whileTrue(Commands.defer(() -> {
        // final var alliance = DriverStation.getAlliance().get();
        // final var speaker = alliance == Alliance.Blue ? AutoComponents.BLUE_SPEAKER
        // : AutoComponents.RED_SPEAKER;
        // return new MovingAutoAimCommand(speaker);
        // }, Set.of()));

        mOperatorController.rightTrigger(kOperatorTriggerDeadband)
                .onTrue(mShooter.setShooterStateCommand(ShooterState.ON))
                .whileTrue(DigestCommands.in(true))
                .onFalse(mShooter.setShooterStateCommand(ShooterState.OFF));
        // mOperatorController.a().onTrue(mShooter.setShooterStateCommand(ShooterState.OFF));
        // mOperatorController.y().onTrue(mShooter.setShooterStateCommand(ShooterState.ON));

        // mOperatorController.x().onTrue(mShooter.setConveyorStateCommand(ConveyorState.OFF));
        // mOperatorController.b().onTrue(mShooter.setConveyorStateCommand(ConveyorState.IN));

        // mOperatorController.button(10).whileTrue(NoteVisualizer.visualizeTrajectoryCommand());

        // mOperatorController.button(13)
        // .whileTrue(new
        // MovingAutoAimCommand(com.spartronics4915.frc2024.Constants.AutoAimConstants.kAutoAimTarget));

        // mDriverController.a()
        // .whileTrue(new ToggleDetectorCommand());

        // mOperatorController.button(15).onTrue(mSwerveDrive.toggleFieldRelativeCommand());

    }

    public Command getAutonomousCommand() {
        // return AutoBuilder.followPath(PathPlannerPath.fromPathFile("CenterToFirstRowTop"));

        // return mShooter.setShooterStateCommand(ShooterState.ON)
        // .andThen(AutoBuilder.buildAuto("CenterRow4NoteKickoffOnly"));
        // return mShooter.setShooterStateCommand(ShooterState.ON)
        // .andThen(AutoBuilder.buildAuto("FastCenterRow4NoteKickoffOnly"));

        // return mShooter.setShooterStateCommand(ShooterState.ON)
        //         .andThen(AutoBuilder.buildAuto("MiddleLower2Kickoff"));

        return mAutoChooser.getSelected();

        // return mAutoChooser.getSelected();
    }

    private SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> out = new SendableChooser<Command>();

        out.setDefaultOption("None", Commands.none());
        
        out.addOption("Preloaded only", AutoComponents.shootPreloaded());
        
        out.addOption("Center 4-note", StartingPosition.setStartingPosition(StartingPosition.MIDDLE).andThen(CompositeAutos.generateCenterFourNote()));

        out.addOption("Center 4-note faster", StartingPosition.setStartingPosition(StartingPosition.MIDDLE).andThen(CompositeAutos.generateCenterFourNoteFaster()));

        out.addOption("Middle Lower 2", mShooter.setShooterStateCommand(ShooterState.ON)
                .andThen(AutoBuilder.buildAuto("MiddleLower2Kickoff")));

        return out;
    }

    public SwerveDrive getSwerveDrive() {
        return mSwerveDrive;
    }

    private static class CompositeAutos {

        public static Command generateCenterFourNote() {
            PathSet pickUpFinalNote = new AutoFactory.PathSet(
                    PathPlannerPath.fromPathFile("entry 3"),
                    PathPlannerPath.fromPathFile("sweep 3")).withNoteApproachParams(-18, 0.5); // .withMinTyForSweepPath(5);
            return AutoFactory.generateVisionAuto(
                    new AutoFactory.PathSet(
                            PathPlannerPath.fromPathFile("CenterToFirstRowTop")),
                    new AutoFactory.PathSet(
                            PathPlannerPath.fromPathFile("entry 2")),
                    pickUpFinalNote);
        }

        public static Command generateCenterFourNoteFaster() {
            PathSet pickUpFinalNote = new AutoFactory.PathSet(
                    PathPlannerPath.fromPathFile("entry 3"),
                    PathPlannerPath.fromPathFile("sweep 3")).withNoteApproachParams(-18, 0.5); // .withMinTyForSweepPath(5);
            return AutoFactory.generateVisionAuto(
                    new AutoFactory.PathSet(
                            PathPlannerPath.fromPathFile("CenterToFirstRowTop"))
                                    .withNoteApproachForwardVelocity(1.5)
                                    .withNoteApproachParams(-18, 0.85),
                    new AutoFactory.PathSet(
                            PathPlannerPath.fromPathFile("entry2Fast")),
                    pickUpFinalNote);
        }

        public static Command lowerTwoNote() {
            PathPlannerPath entryPath = PathPlannerPath.fromPathFile("entryMiddle4");
            PathPlannerPath returnPath = PathPlannerPath.fromPathFile("exitMiddle4");
            PathPlannerPath entryPath5 = PathPlannerPath.fromPathFile("entry5");
            PathPlannerPath returnPath5 = PathPlannerPath.fromPathFile("ExitMiddle5");

            return Commands.sequence(
                    AutoComponents.shootPreloaded(),
                    AutoBuilder.followPath(entryPath),
                    Commands.parallel(AutoComponents.groundIntake(),
                            LimelightAuto.driveToNote(1, Optional.of(Double.valueOf(-18)),
                                    Optional.of(Double.valueOf(0.5)))),
                    AutoBuilder.followPath(returnPath),
                    AutoFactory.loadAndAimCommand(),
                    AutoComponents.shootFromLoaded(),
                    AutoBuilder.followPath(entryPath5),
                    Commands.parallel(AutoComponents.groundIntake(),
                            LimelightAuto.driveToNote(1, Optional.of(Double.valueOf(-18)),
                                    Optional.of(Double.valueOf(0.5)))),
                    AutoBuilder.followPath(returnPath5),
                    AutoFactory.loadAndAimCommand(),
                    AutoComponents.shootFromLoaded());

        }

    }

}
