package com.spartronics4915.frc2024.commands;

import java.nio.channels.ShutdownChannelGroupException;

import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.commands.drivecommands.DriveStraightCommands;
import com.spartronics4915.frc2024.subsystems.Shooter;
import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.subsystems.Shooter.ConveyorState;
import com.spartronics4915.frc2024.subsystems.Shooter.ShooterState;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LimelightAuto {
    private static final VisionSubsystem mVisionSubsystem = VisionSubsystem.getInstance();
    private static final SwerveDrive mSwerve = SwerveDrive.getInstance();
    private static final ShooterWrist mShooterWrist = ShooterWrist.getInstance();
    private static final Shooter mShooter = Shooter.getInstance();

    private LimelightAuto() {}

    public static Command limelightAuto() {
        return Commands.sequence(
                IntakeAssemblyCommands.ComplexSetState(IntakeAssemblyState.GROUNDPICKUP),
                driveToNote(),
                new AlignToSpeakerCommand(),
                aimAndShoot());
    }

    public static Command driveToNote() {
        return Commands.deadline(
                Commands.waitUntil(mVisionSubsystem::aliceCantSeeNote), // This shop stop when Alice can't see the note.
                Commands.parallel(
                        new LockOnCommand(),
                        new DriveStraightCommands.DriveStraightFixedDistance(
                                mSwerve,
                                new Rotation2d(),
                                10,
                                new Constraints(1, 1))));
    }

    // TODO: make
    public static Rotation2d getNeededShooterAngle(double tagSize) {
        return new Rotation2d();
    }

    public static Command aimAndShoot() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    mShooterWrist.publicSetRotationSetPoint(getNeededShooterAngle(0));
                }),
                Commands.waitUntil(mShooterWrist::atTarget),
                mShooter.setShooterStateCommand(ShooterState.ON),
                mShooter.setConveyorStateCommand(ConveyorState.IN));
    }
}
