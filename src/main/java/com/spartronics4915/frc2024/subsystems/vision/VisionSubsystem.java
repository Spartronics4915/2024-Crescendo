package com.spartronics4915.frc2024.subsystems.vision;

import java.util.Optional;

import com.spartronics4915.frc2024.subsystems.vision.LimelightDevice.VisionMeasurement;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem mInstance;

    private final LimelightDevice alice;
    private final LimelightDevice bob;
    
    private VisionSubsystem() {
        alice = new LimelightDevice("limelight-alice");
        bob = new LimelightDevice("limelight-bob");
    }

    public static VisionSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new VisionSubsystem();
        }
        return mInstance;
    }

    /**
     * Gets the preferred limelight, which is decided based on whichever limelight sees the most april tags. 
     * If they see the same number of tags, alice is returned.
     * @return The preferred limelight
     */
    private LimelightDevice getPreferredLimelight() {
        int aliceTagCount = alice.numberOfTagsSeen();
        int bobTagCount = bob.numberOfTagsSeen();
        if (aliceTagCount < bobTagCount) return bob;
        return alice;
    }

    /**
     * Returns the Vision Measurement from the preferred limelight, or null if no tags are visible.
     * @return The {@link VisionMeasurement} of the robot, or null
     */
    public Optional<VisionMeasurement> getVisionMeasurement() {
        LimelightDevice preferred = getPreferredLimelight();
        return preferred.getVisionMeasurement();
    }

    /**
     * Returns the current botpose from the preferred limelight, or null if no tags are visible.
     * @return The {@link Pose3d} of the robot, or null
     */
    public Optional<Pose3d> getPose() {
        LimelightDevice preferred = getPreferredLimelight();
        if (preferred.canSeeTags()) return Optional.of(preferred.getBotPose3d());
        return Optional.empty();
    }
    /**
     * @return The greatest number of tags seen from either limelight
     */
    public int getTagCount() {
        int aliceTagCount = alice.numberOfTagsSeen();
        int bobTagCount = bob.numberOfTagsSeen();
        return Math.max(aliceTagCount, bobTagCount);
    }
    public LimelightDevice getAlice() {
        return alice;
    }
    public LimelightDevice getBob() {
        return bob;
    }

    @Override
    public void periodic() {
        getAlice().updateFieldPose();
        getBob().updateFieldPose();
    }
}
