package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAssemblyCommands {
    private IntakeWrist mWrist;
    private Intake mIntake;
    private SubsystemBase mElevator; //FIXME placeholder
    
    public IntakeAssemblyCommands(IntakeWrist mWrist, Intake mIntake) {
        this.mWrist = mWrist;
        this.mIntake = mIntake;
    }

    public Command setState(IntakeAssemblyState newState){
        return Commands.parallel(
            Commands.print("elevator do stuff"),
            mWrist.setStateCommand(newState)
        );
    }

    public boolean atTarget(){
        return mWrist.atTargetState() && true; //FIXME add elevator
    }
    

}
