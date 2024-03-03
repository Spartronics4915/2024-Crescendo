package com.spartronics4915.frc2024.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;

import static com.spartronics4915.frc2024.Constants.AutoAimConstants.*;

import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;

public class StationaryAutoAimCommand extends Command {
    public final Translation3d target;
    private static final SwerveDrive SWERVE = SwerveDrive.getInstance();

    public StationaryAutoAimCommand(Translation3d target) {
        this.target = target;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Rotation2d swerveAngle;
        Rotation2d shooterAngle;

        var cpose = SWERVE.getPose();
        var ct2d = cpose.getTranslation();
        var ct3d = new Translation3d(ct2d.getX(), ct2d.getY(), kShooterHeight);
        var diff = target.minus(ct3d);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
