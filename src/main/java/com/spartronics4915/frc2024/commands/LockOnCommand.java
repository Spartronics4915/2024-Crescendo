package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;
import com.spartronics4915.frc2024.subsystems.vision.LimelightDevice;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class LockOnCommand extends Command {

    private final SwerveDrive mSwerve;
    private final VisionSubsystem mVision;
    private final LimelightDevice mLimelight;

    public LockOnCommand() {
        super();
        mSwerve = SwerveDrive.getInstance();
        mVision = VisionSubsystem.getInstance();
        mLimelight = mVision.getAlice();
    }

    private double getTx() {
        if (mLimelight.getTv()) {
            return mLimelight.getTx();
        } else {
            return 0.0;
        }
    }

    @Override
    public void initialize() {
        mSwerve.decoupleRotation();
    }
    
    @Override
    public void execute() {
        mSwerve.setDesiredAngle(mSwerve.getAngle().rotateBy(Rotation2d.fromDegrees(-getTx())));
        // System.out.println(getTx());
    }

    @Override
    public void end(boolean interrupted) {
        mSwerve.recoupleRotation();
    }
}
