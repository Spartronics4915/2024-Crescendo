package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.subsystems.Elevator;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAssemblyCommands {
    private IntakeWrist mWrist;
    private Intake mIntake;
    private Elevator mElevator;
    
    public IntakeAssemblyCommands(IntakeWrist mWrist, Intake mIntake, Elevator mElevator) {
        this.mWrist = mWrist;
        this.mIntake = mIntake;
        this.mElevator = mElevator;
    }

    public Command setState(IntakeAssemblyState newState){
        return Commands.parallel(
            mElevator.setTargetCommand(newState),
            mWrist.setStateCommand(newState)
        );
    }

    public boolean atTarget(){
        return mWrist.atTargetState(0.015) && mElevator.atTargetState(0.015);
    }
    

}
