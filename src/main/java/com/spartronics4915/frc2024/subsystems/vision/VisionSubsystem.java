package com.spartronics4915.frc2024.subsystems.vision;

import java.util.Optional;

import com.spartronics4915.frc2024.LimelightHelpers;
import com.spartronics4915.frc2024.commands.BootCoralCommand;
import com.spartronics4915.frc2024.commands.visionauto.NoteLocatorInterface;
import com.spartronics4915.frc2024.commands.visionauto.NoteLocatorLimeLight;
import com.spartronics4915.frc2024.commands.visionauto.NoteLocatorSim;
import com.spartronics4915.frc2024.commands.visionauto.SpeakerTargetTagLocatorLimeLight;
import com.spartronics4915.frc2024.commands.visionauto.SpeakerTargetTagLocatorSim;
import com.spartronics4915.frc2024.commands.visionauto.TargetDetectorInterface;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem mInstance;

    private final LimelightDevice alice;
    private final LimelightDevice bob;
    private TargetDetectorInterface noteLocator;
    private TargetDetectorInterface speakerTagLocator;

    private VisionSubsystem() {
        alice = new LimelightDevice("alice", true);
        bob = new LimelightDevice("bob", false);

        if (RobotBase.isSimulation()) {
            SwerveDrive swerve = SwerveDrive.getInstance();
            noteLocator = new NoteLocatorSim(swerve);
            speakerTagLocator = new SpeakerTargetTagLocatorSim(swerve);

        } else {
            noteLocator = new NoteLocatorLimeLight(this);
            speakerTagLocator = new SpeakerTargetTagLocatorLimeLight(bob);
        }
        
        new Trigger(DriverStation::isEnabled).onTrue(Commands.runOnce(this::setBobPriorityTagToSpeaker));
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
        return noteLocator.getClosestVisibleTarget().isPresent();
    }

    public boolean aliceDoesNotSeeNote() {
        return !noteLocator.getClosestVisibleTarget().isPresent();
    }

    public TargetDetectorInterface getNoteLocator() {
        return noteLocator;
    }

    public TargetDetectorInterface getSpeakerTagLocator() {
        return speakerTagLocator;
    }

    public void setBobPriorityTagToSpeaker() {
        int priorityTag = -1;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isEmpty()) {
            if (alliance.get() == Alliance.Blue) priorityTag = 7; // Center of blue speaker
            else if (alliance.get() == Alliance.Red) priorityTag = 4; // Center of red speaker
        }
        bob.setPriorityTagID(priorityTag);
    }

    @Override
    public void periodic() {
        alice.updateFieldPose();
        bob.updateFieldPose();
        alice.profilePipelineSwitching();
        bob.profilePipelineSwitching();
        if (!alice.isValid())
            alice.checkIfValid();
        if (!bob.isValid())
            bob.checkIfValid();
    }
}
