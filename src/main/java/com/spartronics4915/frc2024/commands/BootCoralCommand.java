package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.Constants.Vision.VisionPipelines;
import com.spartronics4915.frc2024.subsystems.vision.LimelightDevice;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class BootCoralCommand extends Command {
    private final LimelightDevice mLimelight;

    public BootCoralCommand() {
        mLimelight = VisionSubsystem.getInstance().getAlice();
    }

    @Override
    public void initialize() {
        mLimelight.setVisionPipeline(VisionPipelines.DETECTOR_NOTE);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        mLimelight.setVisionPipeline(VisionPipelines.FIDUCIALS_3D);
    }
}
