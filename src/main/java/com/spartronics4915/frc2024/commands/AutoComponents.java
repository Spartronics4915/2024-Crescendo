package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake.IntakeState;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoComponents {
    private SwerveDrive mDriveBase;
    private Intake mIntake;
    private IntakeAssemblyCommands mIntakeAssmebly;
    
    private SubsystemBase mShooter; //TODO placeholder
    
    public AutoComponents(SwerveDrive mDriveBase, Intake mIntake, IntakeAssemblyCommands mIntakeAssmebly ) {
        this.mDriveBase = mDriveBase;
        this.mIntake = mIntake;
        this.mIntakeAssmebly = mIntakeAssmebly;
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

    public Command groundToIntake(){
        return Commands.sequence(
            mIntakeAssmebly.setState(IntakeAssemblyState.GROUNDPICKUP),
            Commands.waitUntil(mIntakeAssmebly::atTarget),

            mIntake.setStateCommand(IntakeState.IN),
            Commands.waitUntil(mIntake::getBeamBreakStatus),

            Commands.parallel(
                mIntake.setStateCommand(IntakeState.OFF),
                mIntakeAssmebly.setState(IntakeAssemblyState.LOAD)
            )
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
