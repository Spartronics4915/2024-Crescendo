package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.subsystems.Elevator;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake.IntakeState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAssemblyCommands {
    private static IntakeWrist mWrist = IntakeWrist.getInstance();
    private static Intake mIntake = Intake.getInstance();
    private static Elevator mElevator = Elevator.getInstance();

    private IntakeAssemblyCommands() {};

    public static Command home() {
        return new HomingCommand();
    }

    public static Command stow() {
        return home().andThen(setState(IntakeAssemblyState.STOW));
    }

    public static Command setState(IntakeAssemblyState newState){
        return Commands.parallel(
            mElevator.setTargetCommand(newState),
            mWrist.setStateCommand(newState)
        );
    }

    public static boolean atTarget() {
        System.out.println("Calling Intake Assembly at target");
        return mWrist.atTargetState(0.015) && mElevator.atTargetState(0.01);
    }
    
    public static Command ComplexSetState(IntakeAssemblyState newState){

        return Commands.parallel(
            setState(newState),
            switch (newState) {
                case SOURCE -> mIntake.setStateCommand(IntakeState.IN);
                case GROUNDPICKUP -> mIntake.setStateCommand(IntakeState.IN);
                default -> mIntake.setStateCommand(IntakeState.OFF);
            }
        );
    }

}
