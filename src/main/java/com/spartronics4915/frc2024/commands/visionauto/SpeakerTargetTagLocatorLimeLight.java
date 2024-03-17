package com.spartronics4915.frc2024.commands.visionauto;

import com.spartronics4915.frc2024.subsystems.vision.LimelightDevice;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;


public class SpeakerTargetTagLocatorLimeLight implements TargetDetectorInterface{
    
    LimelightDevice limelight;
    final double TAG_HEIGHT;
    final double CAMERA_HEIGHT = 0;
    public SpeakerTargetTagLocatorLimeLight(LimelightDevice limelight) {
        this.limelight = limelight;

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

        TAG_HEIGHT = fieldLayout.getTagPose(7).get().getTranslation().getZ();
    }

    public Optional<Detection> getClosestVisibleTarget() {

        if(!limelight.getTv()) {
            return Optional.empty();
        }

        Rotation2d ty = Rotation2d.fromDegrees(-limelight.getTy());
        double estimatedDistance = (TAG_HEIGHT - CAMERA_HEIGHT) / ty.getTan();

        return Optional.of(new Detection(limelight.getTx(), limelight.getTy(), estimatedDistance));

    }
}
