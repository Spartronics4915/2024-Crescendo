package com.spartronics4915.frc2024.subsystems.vision;

import com.spartronics4915.frc2024.commands.BootCoralCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem mInstance;

    private final LimelightDevice alice;
    private final LimelightDevice bob;

    private VisionSubsystem() {
        alice = new LimelightDevice("alice", true);
        bob = new LimelightDevice("bob", false);
    }
    
    public static VisionSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new VisionSubsystem();
            // BootCoralCommand bootCoral = new BootCoralCommand();
            // CommandScheduler.getInstance().schedule(bootCoral);
        }
        return mInstance;
    }

    public LimelightDevice getAlice() {
        return alice;
    }

    public LimelightDevice getBob() {
        return bob;
    }

    public boolean aliceSeesNote() {
        return alice.getTv();
    }

    @Override
    public void periodic() {
        alice.updateFieldPose();
        bob.updateFieldPose();
        alice.profilePipelineSwitching();
        bob.profilePipelineSwitching();
        if (!alice.isValid()) alice.checkIfValid();
        if (!bob.isValid()) bob.checkIfValid();
    }
}
