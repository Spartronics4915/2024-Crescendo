package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.Constants.Vision.VisionPipelines;
import com.spartronics4915.frc2024.subsystems.vision.LimelightDevice;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class BootCoralCommand extends Command {
    private final LimelightDevice mLimelight;
    private double bootTime;
    private boolean booting = false;
    private boolean booted = false;

    public BootCoralCommand() {
        mLimelight = VisionSubsystem.getInstance().getAlice();
    }

    @Override
    public void initialize() {
        // mLimelight.setVisionPipeline(VisionPipelines.DETECTOR_NOTE);
        System.out.println("[BOOTCORAL] init");
    }

    @Override
    public void execute() {
        if (mLimelight.isValid()) {
            if (!booting) {
                mLimelight.setVisionPipeline(VisionPipelines.DETECTOR_NOTE);
                booting = true;
                bootTime = Timer.getFPGATimestamp();
                System.out.println("[BOOTCORAL] booting at " + bootTime);
            } else if (!booted) {
                double currentTime = Timer.getFPGATimestamp();
                double timeElapsed = currentTime - bootTime;
                if (mLimelight.getTruePipelineIndex() == 1 || (timeElapsed > 8)) {
                    booted = true;
                    System.out.println("[BOOTCORAL] booted in " + timeElapsed);
                    if (timeElapsed > 8) System.out.println("[BOOTCORAL] timed out on bootup");
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return booted;
        // return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[BOOTCORAL] end");
        mLimelight.setVisionPipeline(VisionPipelines.FIDUCIALS_3D);
    }
}
