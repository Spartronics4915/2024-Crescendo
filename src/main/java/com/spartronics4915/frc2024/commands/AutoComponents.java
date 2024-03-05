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
        final var alliance = DriverStation.getAlliance().get();
        final var speaker = alliance == Alliance.Blue ? BLUE_SPEAKER : RED_SPEAKER;
        return Optional.of(speaker);
    }

    public static Command shootPreloaded() {
        return Commands.parallel(
                mShooterWrist.setStateCommand(ShooterWristState.SUBWOOFER_SHOT),
                mShooter.setShooterStateCommand(ShooterState.ON).withTimeout(2)
                    .andThen(DigestCommands.in().withTimeout(5)));
    }

    public static Command loadIntoShooter() {
        return Commands.sequence(
                Commands.parallel(
                        mIntake.setStateCommand(IntakeState.OFF),
                        IntakeAssemblyCommands.setState(IntakeAssemblyState.LOAD)),
                Commands.waitUntil(IntakeAssemblyCommands::atTarget),
                mIntake.setStateCommand(IntakeState.LOAD),
                mShooter.setConveyorStateCommand(ConveyorState.IN),
                // Commands.waitUntil(() -> {
                // return mIntake.getBeamBreakStatus();
                // }),
                Commands.waitSeconds(4),
                mIntake.setStateCommand(IntakeState.OFF));
    }

    public static Command shootFromLoaded() {
        return Commands.sequence(
                mShooter.setShooterStateCommand(ShooterState.ON),
                Commands.waitUntil(mShooter::hasSpunUp),
                mShooter.setConveyorStateCommand(ConveyorState.IN));
    }

    public static Command shooterAim(Supplier<Rotation2d> aimSupplier) {
        return Commands.sequence(
                Commands.deadline(
                        Commands.waitUntil(() -> mShooter.hasSpunUp() && mShooterWrist.atTarget()),
                        mShooter.setShooterStateCommand(ShooterState.ON),
                        mShooterWrist.angleToSupplierCommand(aimSupplier)));
    }

    public static Command groundToIntake() {
        return Commands.sequence(
                IntakeAssemblyCommands.ComplexSetState(IntakeAssemblyState.GROUNDPICKUP),
                Commands.waitUntil(IntakeAssemblyCommands::atTarget),
                Commands.waitUntil(mIntake::getBeamBreakStatus),
                IntakeAssemblyCommands.ComplexSetState(IntakeAssemblyState.LOAD));
    }

    public static Command resetToGround() {
        return Commands.sequence(
                Commands.parallel(
                        mIntake.setStateCommand(IntakeState.OFF),
                        IntakeAssemblyCommands.setState(IntakeAssemblyState.GROUNDPICKUP)),
                Commands.waitUntil(IntakeAssemblyCommands::atTarget));
    }

    /**
     * do not use please
     */
    @Deprecated
    public static Command stationaryAutoAim() {
        return Commands.defer(() -> {
            final var alliance = DriverStation.getAlliance().get();
            final var speaker = alliance == Alliance.Blue ? BLUE_SPEAKER : RED_SPEAKER;
            final var aac = new MovingAutoAimCommand(speaker);
            return Commands.deadline(Commands.waitUntil(aac::atTarget), aac);
        }, Set.of(mSwerve, mShooterWrist));
    }

    public static Command stationaryAimAndShootSequential() {
        return Commands.sequence(stationaryAutoAim(), shootFromLoaded());
    }

    public static Command stationaryAimAndShootParallel() {
        return Commands.parallel(Commands.waitUntil(mShooter::hasSpunUp), stationaryAutoAim())
                .andThen(shootFromLoaded());
    }
}
