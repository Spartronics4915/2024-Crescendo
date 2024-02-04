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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.*;

public class AutoComponents {
    private SwerveDrive mDriveBase;
    private Intake mIntake;
    private IntakeAssemblyCommands mIntakeAssmebly;
    
    private Shooter mShooter; //TODO placeholder
    private ShooterWrist mShooterWrist;

    public AutoComponents(SwerveDrive mDriveBase, Intake mIntake, IntakeAssemblyCommands mIntakeAssmebly, Shooter mShooter, ShooterWrist mShooterWrist) {
        this.mDriveBase = mDriveBase;
        this.mIntake = mIntake;
        this.mIntakeAssmebly = mIntakeAssmebly;
        this.mShooter = mShooter;
        this.mShooterWrist = mShooterWrist;
    }
    
    public Command loadIntoShooter(){
        return Commands.sequence(
            Commands.parallel(
                mIntake.setStateCommand(IntakeState.OFF),
                mIntakeAssmebly.setState(IntakeAssemblyState.LOAD)
            ),
            Commands.waitUntil(mIntakeAssmebly::atTarget),

            mIntake.setStateCommand(IntakeState.LOAD),
            Commands.waitUntil(() -> {return !mIntake.getBeamBreakStatus();}),

            mIntake.setStateCommand(IntakeState.OFF)
            //TODO add shooter holding code
        );
    }

    public Command shootFromLoaded(){
        return Commands.sequence(
            mShooter.setShooterStateCommand(ShooterState.ON),
            Commands.waitUntil(mShooter::hasSpunUp),
            mShooter.setConveyorStateCommand(ConveyorState.IN)
        );
    }

    public Command shooterAim(Supplier<Rotation2d> aimSupplier){
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> mShooter.hasSpunUp() && mShooterWrist.atTarget()),
                mShooter.setShooterStateCommand(ShooterState.ON),
                mShooterWrist.angleToSupplierCommand(aimSupplier)
            )
        );
    }

    public Command groundToIntake(){
        return Commands.sequence(
            mIntakeAssmebly.ComplexSetState(IntakeAssemblyState.GROUNDPICKUP),
            Commands.waitUntil(mIntakeAssmebly::atTarget),

            Commands.waitUntil(mIntake::getBeamBreakStatus),
            mIntakeAssmebly.ComplexSetState(IntakeAssemblyState.LOAD)
        );
    }

    public Command resetToGround(){
        return Commands.sequence(
            Commands.parallel(
                mIntake.setStateCommand(IntakeState.OFF),
                mIntakeAssmebly.setState(IntakeAssemblyState.GROUNDPICKUP)
            ),
            Commands.waitUntil(mIntakeAssmebly::atTarget)
        );
    }

    public Command AimAndShoot(){
        return Commands.none(); //TODO placeholder, shooter and swerve
    }

}
