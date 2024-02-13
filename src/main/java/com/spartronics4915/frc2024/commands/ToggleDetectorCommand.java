package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.Constants.Vision.VisionPipelines;
import com.spartronics4915.frc2024.subsystems.vision.LimelightDevice;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleDetectorCommand extends Command {
    private final LimelightDevice mLimelight;

    public ToggleDetectorCommand() {
        mLimelight = VisionSubsystem.getInstance().getAlice();
    }

    @Override
    public void initialize() {
        System.out.println("toggle init");
        mLimelight.setVisionPipeline(VisionPipelines.DETECTOR_NOTE);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("toggle end");
        mLimelight.setVisionPipeline(VisionPipelines.FIDUCIALS_3D);
    }
}
