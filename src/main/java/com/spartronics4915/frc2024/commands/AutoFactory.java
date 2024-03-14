package com.spartronics4915.frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.spartronics4915.frc2024.RobotContainer;
import com.spartronics4915.frc2024.subsystems.Shooter;
import com.spartronics4915.frc2024.subsystems.Shooter.ShooterState;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import static com.spartronics4915.frc2024.Constants.Drive.kMaxSpeed;
import static com.spartronics4915.frc2024.Constants.Drive.kMaxAcceleration;
import static com.spartronics4915.frc2024.Constants.Drive.kMaxAngularSpeed;
import static com.spartronics4915.frc2024.Constants.Drive.kMaxAngularAcceleration;

public final class AutoFactory {
   
    public static final class PathSet {
        PathPlannerPath drivePath;
        Optional<PathPlannerPath> sweepPath;
        Optional<Double> noteApproachSlowSpeed;
        Optional<Double> noteApproachSlowThreshold;
        Optional<PathPlannerPath> returnToStartSweepPath;
        double minTyForSweepPath;

        public PathSet(PathPlannerPath drivePath) {
            this.drivePath = drivePath;
            this.sweepPath = Optional.empty();
            noteApproachSlowSpeed = Optional.empty();
            noteApproachSlowThreshold = Optional.empty();
            returnToStartSweepPath = Optional.empty();
            minTyForSweepPath = 0;

        }

        public PathSet(PathPlannerPath drivePath, PathPlannerPath sweepPath) {
            this(drivePath);
            this.sweepPath = Optional.of(sweepPath);

        }

        public PathSet withNoteApproachParams(double noteApproachSlowThreshold, double noteApproachSlowSpeed) {

            this.noteApproachSlowThreshold = Optional.of(Double.valueOf(noteApproachSlowThreshold));
            this.noteApproachSlowSpeed = Optional.of(Double.valueOf(noteApproachSlowSpeed));
            return this;
        }

        public PathSet withReturnToStartSweepPath(PathPlannerPath returnPath) {
            this.returnToStartSweepPath = Optional.of(returnPath);
            return this;
        }

        public PathSet withMinTyForSweepPath(double minTy) {
            this.minTyForSweepPath = minTy;
            return this;
        }

        PathPlannerPath drivePath() { return drivePath;}
        Optional<PathPlannerPath> sweepPath() { return sweepPath;};

    }

    private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
            kMaxSpeed, kMaxAcceleration, kMaxAngularSpeed, kMaxAngularAcceleration);

    private AutoFactory() {}

    public static Command generateVisionAuto(PathSet... paths) {
        return generateVisionAuto(List.of(paths));
    }

    public static Command generateVisionAuto(List<PathSet> paths) {
        final Command[] seq = paths.stream().map((ps) -> generateAutoSegment(ps)).toArray(Command[]::new);
        return AutoComponents.shootPreloaded().andThen(Commands.sequence(seq));
    }

    public static Command generateBlindAuto(PathPlannerPath... paths) {
        return generateBlindAuto(List.of(paths));
    }

    public static Command generateBlindAuto(List<PathPlannerPath> paths) {
        return Commands.print("generateBlindAuto not yet implemented!");
    }

    private static Command generateAutoSegment(PathSet pathSet) {
        final var sp = pathSet.sweepPath();
        final Command aimCommand = sp.isPresent() ? generateSweepCommand(pathSet) : loadAndAimCommand();
        return Commands.sequence(
                generateDriveCommand(pathSet),
                aimCommand,
                shootCommand());
    }

    private static Command generateDriveCommand(PathSet pathSet) {
        boolean groundIntake = true;
        final Command intakeAction = groundIntake ? AutoComponents.groundIntake() : Commands.none();
        final Command noteAction = groundIntake
                ? LimelightAuto.driveToNote(pathSet.noteApproachSlowThreshold, pathSet.noteApproachSlowSpeed)
                : Commands.none();
        return generateDriveCommand(pathSet.drivePath, intakeAction, noteAction);

    }

    private static Command generateNoPauseToShootAutoSegment(PathSet pathSet) {

        Command driveSweepPathCommand = AutoBuilder.followPath(pathSet.sweepPath.get());
        return Commands.sequence(generateDriveCommand(pathSet.drivePath),
                AutoComponents.loadWhileDrivingThenStopAndShootIfNoteLoaded(driveSweepPathCommand));
    }

    private static Command generateDriveCommand(PathPlannerPath path) {
        boolean groundIntake = true;
        final Command intakeAction = groundIntake ? AutoComponents.groundIntake() : Commands.none();
        final Command noteAction = groundIntake ? LimelightAuto.driveToNote() : Commands.none();
        return generateDriveCommand(path, intakeAction, noteAction);
    }

    private static Command generateDriveCommand(PathPlannerPath path, Command intakeAction, Command noteAction) {
        return Commands.parallel(
                intakeAction,
                Commands.sequence(
                        AutoBuilder.followPath(path),
                        Commands.waitUntil(VisionSubsystem.getInstance()::aliceSeesNote).withTimeout(1),
                        noteAction));
    }

    private static Command generateSweepCommand(PathSet pathSet) {
        PathPlannerPath path = pathSet.sweepPath.get();
        Command sweepSequence =  Commands.sequence(
                RobotContainer.getShooterFireControl().initRunCommand(),
                Commands.parallel(
                        AutoComponents.loadIntoShooter(),
                        Commands.sequence(
                                Commands.race(
                                        AutoBuilder.pathfindThenFollowPath(path, PATH_CONSTRAINTS),
                                        RobotContainer.getShooterFireControl().trackRunCommand(pathSet.minTyForSweepPath)).andThen(Commands.waitSeconds(0.3)),
                                SwerveDrive.getInstance().stopCommand(),
                                Shooter.getInstance().setShooterStateCommand(ShooterState.ON),
                                AutoComponents.stationaryAutoAim().withTimeout(2))));

        if(pathSet.returnToStartSweepPath.isEmpty()) {
            return sweepSequence;
        } else {
            return AutoBuilder.followPath(pathSet.returnToStartSweepPath.get()).andThen(sweepSequence);
        }
    }

    private static Command loadAndAimCommand() {
        return Commands.parallel(
                AutoComponents.loadIntoShooter(),
                Shooter.getInstance().setShooterStateCommand(ShooterState.ON),
                AutoComponents.stationaryAutoAim().withTimeout(2));
    }

    private static Command shootCommand() {
        return AutoComponents.shootFromLoaded();
    }
}
