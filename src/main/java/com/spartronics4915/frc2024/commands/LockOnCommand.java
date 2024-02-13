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
        //do thing if rotation already decoupled
        mSwerve.decoupleRotation();
        mLimelight.setVisionPipeline(VisionPipelines.DETECTOR_NOTE);
    }
    
    @Override
    public void execute() {
        double pipelineIndex = mLimelight.getTruePipelineIndex();
        Rotation2d swerveAngle = mSwerve.getAngle();
        Rotation2d limelightAngle = Rotation2d.fromDegrees(-mLimelight.getTx());
        Rotation2d desiredAngle = Rotation2d.fromRotations(swerveAngle.getRotations() + limelightAngle.getRotations());
        System.out.println("LOCKON:\nswerve: " + swerveAngle + "\nlimelight: " + limelightAngle + "\ndesired: " + desiredAngle);
        if ((pipelineIndex == 1) && mLimelight.getTv()) mSwerve.setDesiredAngle(desiredAngle);
    }

    @Override
    public void end(boolean interrupted) {
        mSwerve.recoupleRotation();
        System.out.println("LOCKON:\nended; interrupted = " + interrupted);
        mLimelight.setVisionPipeline(VisionPipelines.FIDUCIALS_3D);
    }
}
