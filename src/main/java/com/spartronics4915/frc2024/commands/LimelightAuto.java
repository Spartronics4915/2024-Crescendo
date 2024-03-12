package com.spartronics4915.frc2024.commands;

import java.nio.channels.ShutdownChannelGroupException;

import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.commands.drivecommands.DriveStraightCommands;
import com.spartronics4915.frc2024.subsystems.Shooter;
import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.subsystems.Shooter.ConveyorState;
import com.spartronics4915.frc2024.subsystems.Shooter.ShooterState;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
                Commands.waitSeconds(1),
                driveToNote(),
                Commands.waitSeconds(1),
                new AlignToSpeakerCommand(),
                Commands.waitSeconds(1),
                aimAndShoot());
    }

    public static Command driveToNote() {

        final double FORWARD_VELOCITY = 1;

        ChassisSpeeds forwardSpeed = new ChassisSpeeds(FORWARD_VELOCITY, 0, 0);
        ChassisSpeeds zeroSpeed = new ChassisSpeeds(0, 0, 0);
        // You probably want to just drive straight indefinitely until the command ends.

        Command driveStraightIndefiniteCommand = mSwerve.run(()->{mSwerve.drive(forwardSpeed, false);});
        return Commands.deadline(
                Commands.race(
                    Commands.waitUntil(mVisionSubsystem::aliceDoesNotSeeNote).andThen(Commands.waitSeconds(1)),
                    Commands.waitUntil(Intake.getInstance()::beamBreakIsTriggered)
                ),
                Commands.parallel(
                        new LockOnCommand(mVisionSubsystem.getNoteLocator()),
                        driveStraightIndefiniteCommand)
                ).andThen(mSwerve.runOnce(() -> {mSwerve.drive(zeroSpeed, false);}));
                        // new DriveStraightCommands.DriveStraightFixedDistance(
                        //         mSwerve,
                        //         new Rotation2d(),
                        //         10,
                        //         new Constraints(1, 1))));
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
