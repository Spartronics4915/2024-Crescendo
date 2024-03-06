package com.spartronics4915.frc2024.commands.visionauto;

import java.util.Optional;

import com.spartronics4915.frc2024.subsystems.vision.LimelightDevice;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;

public class NoteLocatorLimeLight implements TargetDetectorInterface {
    private final VisionSubsystem mVision;
    private final LimelightDevice mLimelight;

    public NoteLocatorLimeLight() {
        mVision = VisionSubsystem.getInstance();
        mLimelight = mVision.getAlice();

    }

    public Optional<Detection> getClosestVisibleTarget() {
        final double CAMERA_HEIGHT = 0.3;
        double pipelineIndex = mLimelight.getTruePipelineIndex();
        boolean loaded = pipelineIndex == 1;

        if (!loaded || !mLimelight.getTv()) {
            return Optional.empty();
        }
        double tx = mLimelight.getTxLowpass();
        double ty = mLimelight.getTy();
        double estimatedHeight = -CAMERA_HEIGHT / Rotation2d.fromDegrees(ty).getTan();
        return Optional.of(new Detection(tx, ty, estimatedHeight));
    }

}
