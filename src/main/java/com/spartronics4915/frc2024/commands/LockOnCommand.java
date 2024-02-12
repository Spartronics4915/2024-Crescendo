package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.Constants.Vision.VisionPipelines;
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

    @Override
    public void initialize() {
        mSwerve.decoupleRotation();
        // mLimelight.setVisionPipeline(VisionPipelines.DETECTOR_NOTE); FLASH FIRST
    }
    
    @Override
    public void execute() {
        mSwerve.setDesiredAngle(mSwerve.getAngle().rotateBy(Rotation2d.fromDegrees(-mLimelight.getTx())));
    }

    @Override
    public void end(boolean interrupted) {
        mSwerve.recoupleRotation();
        // mLimelight.setVisionPipeline(VisionPipelines.FIDUCIALS_3D); FLASH FIRST
    }
}
