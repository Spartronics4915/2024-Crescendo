package com.spartronics4915.frc2024.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import static com.spartronics4915.frc2024.Constants.Drive.kMaxSpeed;
import static com.spartronics4915.frc2024.Constants.Drive.kMaxAcceleration;
import static com.spartronics4915.frc2024.Constants.Drive.kMaxAngularSpeed;
import static com.spartronics4915.frc2024.Constants.Drive.kMaxAngularAcceleration;

public final class AutoFactory {
    public static final record PathSet(PathPlannerPath drivePath, Optional<PathPlannerPath> sweepPath) {
        public PathSet(PathPlannerPath drivePath, PathPlannerPath sweepPath) {
            this(drivePath, Optional.of(sweepPath));
        }
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

    private static Command generateAutoSegment(PathSet pathSet) {
        final var sp = pathSet.sweepPath();
        final Command aimCommand = sp.isPresent() ? generateSweepCommand(sp.get()) : loadAndAimCommand();
        return Commands.sequence(
                generateDriveCommand(pathSet.drivePath()),
                aimCommand,
                shootCommand());
    }

    private static Command generateNoPauseToShootAutoSegment(PathSet pathSet) {

        Command driveSweepPathCommand = AutoBuilder.followPath(pathSet.sweepPath.get());
        return Commands.sequence(generateDriveCommand(pathSet.drivePath),
        AutoComponents.loadWhileDrivingThenStopAndShootIfNoteLoaded(driveSweepPathCommand));
    }

    private static Command generateDriveCommand(PathPlannerPath path) {
        return generateDriveCommand(path, true);
    }

    private static Command generateDriveCommand(PathPlannerPath path, boolean groundIntake) {
        final Command intakeAction = groundIntake ? NamedCommands.getCommand("groundIntake") : Commands.none();
        final Command noteAction = groundIntake ? NamedCommands.getCommand("DriveToPickUpNote") : Commands.none();
        return Commands.parallel(
                intakeAction,
                Commands.sequence(
                    AutoBuilder.followPath(path),
                    Commands.waitUntil(VisionSubsystem.getInstance()::aliceSeesNote).withTimeout(1),
                    noteAction));
    }

    private static Command generateSweepCommand(PathPlannerPath path) {
        return Commands.sequence(
                NamedCommands.getCommand("InitShooterFireControl"),
                Commands.parallel(
                    NamedCommands.getCommand("loadIntoShooter"),
                    Commands.sequence(
                        Commands.race(
                            AutoBuilder.pathfindThenFollowPath(path, PATH_CONSTRAINTS),
                            NamedCommands.getCommand("FireControlTracking")),
                        NamedCommands.getCommand("StopChassis"),
                        NamedCommands.getCommand("shooterOn"),
                        NamedCommands.getCommand("stationaryAutoAim"))));
    }

    private static Command loadAndAimCommand() {
        return Commands.parallel(
                NamedCommands.getCommand("loadIntoShooter"),
                NamedCommands.getCommand("shooterOn"),
                NamedCommands.getCommand("stationaryAutoAim"));
    }

    private static Command shootCommand() {
        return NamedCommands.getCommand("shootFromLoaded");
    }
}
