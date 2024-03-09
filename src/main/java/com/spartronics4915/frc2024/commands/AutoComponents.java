package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.Constants.ShooterWristConstants.ShooterWristState;
import com.spartronics4915.frc2024.subsystems.Shooter;
import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake.IntakeState;
import com.spartronics4915.frc2024.subsystems.Shooter.ConveyorState;
import com.spartronics4915.frc2024.subsystems.Shooter.ShooterState;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.Set;
import java.util.function.*;

public class AutoComponents {
    private static SwerveDrive mSwerve = SwerveDrive.getInstance();
    private static Intake mIntake = Intake.getInstance();

    private static Shooter mShooter = Shooter.getInstance();
    private static ShooterWrist mShooterWrist = ShooterWrist.getInstance();

    public static final Translation3d TAG_4 = new Translation3d(Units.inchesToMeters(652.73),
            Units.inchesToMeters(218.42), Units.inchesToMeters(57.13));

    public static final Translation3d TAG_7 = new Translation3d(Units.inchesToMeters(-1.5),
            Units.inchesToMeters(218.42), Units.inchesToMeters(57.13));

    public static final Translation3d RED_SPEAKER = new Translation3d(TAG_4.getX() - Units.inchesToMeters(9),
            TAG_4.getY(), Units.inchesToMeters(86));

    public static final Translation3d BLUE_SPEAKER = new Translation3d(TAG_7.getX() + Units.inchesToMeters(9),
            TAG_7.getY(), Units.inchesToMeters(86));

    private AutoComponents() {};

    public static Translation3d getTargetUnsafe() throws NoSuchElementException {
        return DriverStation.getAlliance().get() == Alliance.Blue ? BLUE_SPEAKER : RED_SPEAKER;
    }

    public static Optional<Translation3d> getTarget() {
        if (DriverStation.getAlliance().isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(getTargetUnsafe());
    }

    public static Command shootPreloaded() {
        return Commands.parallel(
                mShooterWrist.setStateCommand(ShooterWristState.SUBWOOFER_SHOT),
                mShooter.setShooterStateCommand(ShooterState.ON)
                    .andThen(Commands.waitUntil(mShooter::hasSpunUp))
                    .andThen(DigestCommands.in().withTimeout(5)));
    }

    public static Command loadIntoShooter() {
        return Commands.print("loadIntoShooter not implemented: wait until shooter beam break is installed");
    }

    public static Command shootFromLoaded() {
        return Commands.sequence(
                mShooter.setShooterStateCommand(ShooterState.ON),
                Commands.waitUntil(mShooter::hasSpunUp),
                DigestCommands.inUnsafe().withTimeout(1));
    }

    public static Command shooterAim(Supplier<Rotation2d> aimSupplier) {
        return Commands.deadline(
                Commands.waitUntil(() -> mShooter.hasSpunUp() && mShooterWrist.atTarget()),
                mShooter.setShooterStateCommand(ShooterState.ON),
                mShooterWrist.angleToSupplierCommand(aimSupplier));
    }

    public static Command groundIntake() {
        return Commands.sequence(
                IntakeAssemblyCommands.ComplexSetState(IntakeAssemblyState.GROUNDPICKUP),
                Commands.waitUntil(IntakeAssemblyCommands::atTarget),
                Commands.waitUntil(mIntake::beamBreakIsTriggered),
                IntakeAssemblyCommands.setState(IntakeAssemblyState.LOAD));
    }

    public static Command resetToGround() {
        return Commands.sequence(
                Commands.parallel(
                        mIntake.setStateCommand(IntakeState.OFF),
                        IntakeAssemblyCommands.setState(IntakeAssemblyState.GROUNDPICKUP)),
                Commands.waitUntil(IntakeAssemblyCommands::atTarget));
    }

    public static Command stationaryAutoAim() {
        try {
            return Commands.defer(() -> {
                final Translation3d speaker = getTargetUnsafe();
                final StationaryAutoAimCommand aac = new StationaryAutoAimCommand(speaker);
                return Commands.deadline(Commands.waitUntil(aac::atTarget), aac);
            }, Set.of(mSwerve, mShooterWrist));
        } catch (Exception ex) {
            ex.printStackTrace(System.err);
        }
        return Commands.print("Auto aim failed!");
    }

    // TODO: modify to load faster when beam break is added
    public static Command stationaryAimAndShoot() {
        return Commands.sequence(
                mShooter.setShooterStateCommand(ShooterState.ON),
                Commands.parallel(
                    Commands.waitUntil(mShooter::hasSpunUp),
                    stationaryAutoAim()),
                DigestCommands.in());
    }
}
