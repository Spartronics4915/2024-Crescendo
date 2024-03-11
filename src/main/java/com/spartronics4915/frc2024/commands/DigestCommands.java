package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.subsystems.Shooter;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake.IntakeState;
import com.spartronics4915.frc2024.subsystems.Shooter.ConveyorState;
import com.spartronics4915.frc2024.subsystems.Shooter.ShooterState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class DigestCommands {
    private static final Intake mIntake = Intake.getInstance();
    private static final Shooter mShooter = Shooter.getInstance();

    private DigestCommands() {}

    public static Command in() {
        return IntakeAssemblyCommands.setState(IntakeAssemblyState.LOAD)
                .andThen(Commands.waitUntil(IntakeAssemblyCommands::atTarget))
                .andThen(inUnsafe());
    }

    /**
     * Same as {@link in} but without moving the intake to the loading position.
     */
    public static Command inUnsafe() {
        return mIntake.setStateCommand(IntakeState.LOAD)
                .alongWith(mShooter.setConveyorStateCommand(ConveyorState.IN))
                .alongWith(Commands.idle())
                .finallyDo(() -> {
                    mIntake.setState(IntakeState.OFF);
                    mShooter.setConveyorState(ConveyorState.OFF);
                });
    }

    public static Command inFromLoaded() {
        return mIntake.setStateCommand(IntakeState.LOAD)
                .alongWith(mShooter.setConveyorStateCommand(ConveyorState.SHOOTING))
                .alongWith(Commands.idle())
                .finallyDo(() -> {
                    mIntake.setState(IntakeState.OFF);
                    mShooter.setConveyorState(ConveyorState.OFF);
                });
    }

    public static Command out() {
        return mIntake.setStateCommand(IntakeState.OUT)
                .alongWith(mShooter.setConveyorStateCommand(ConveyorState.OUT))
                .alongWith(mShooter.setShooterStateCommand(ShooterState.BACK))
                .alongWith(Commands.idle())
                .finallyDo(() -> {
                    mIntake.setState(IntakeState.OFF);
                    mShooter.setConveyorState(ConveyorState.OFF);
                    mShooter.setShooterState(ShooterState.OFF);
                });
    }
}
