package com.spartronics4915.frc2024.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import com.spartronics4915.frc2024.subsystems.Elevator;
import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;

import edu.wpi.first.math.geometry.Rotation2d;

public class HomingCommand extends ParallelCommandGroup {
    public HomingCommand() {
        super(
                ShooterWrist.getInstance().homeMotorCommand(Rotation2d.fromDegrees(0.3)),
                IntakeWrist.getInstance().homingCommand(Rotation2d.fromDegrees(0.3)),
                Elevator.getInstance().homeMotorCommand(0.01));
    }
}
