package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.subsystems.Shooter;
import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake.IntakeState;
import com.spartronics4915.frc2024.subsystems.Shooter.ConveyorState;
import com.spartronics4915.frc2024.subsystems.Shooter.ShooterState;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.*;

public class AutoComponents {
    private static SwerveDrive mSwerve = SwerveDrive.getInstance();
    private static Intake mIntake = Intake.getInstance();

    private static Shooter mShooter; // TODO placeholder
    private static ShooterWrist mShooterWrist;

    private AutoComponents() {};

    public static Command loadIntoShooter() {
        return Commands.sequence(
                Commands.parallel(
                        mIntake.setStateCommand(IntakeState.OFF),
                        IntakeAssemblyCommands.setState(IntakeAssemblyState.LOAD)),
                Commands.waitUntil(IntakeAssemblyCommands::atTarget),
                mIntake.setStateCommand(IntakeState.LOAD),
                Commands.waitUntil(() -> {
                    return !mIntake.getBeamBreakStatus();
                }),
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
     * 
     * @param aimCalculator takes in the pose of robot, velocity, and outputs a rotation3D
     * @return
     */
    public static Command aimAndShoot(BiFunction<Pose2d, ChassisSpeeds, Rotation3d> aimCalculator) {
        return Commands.none(); // TODO placeholder, shooter and swerve
    }
}
