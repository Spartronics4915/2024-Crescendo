package com.spartronics4915.frc2024.commands.visionauto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.Optional;

public class ShooterRunFireControl {

    TargetDetectorInterface.Detection lastDetection;
    boolean inShootingZone;
    TargetDetectorInterface targetDetector;

    public ShooterRunFireControl(TargetDetectorInterface targetDetector) {

        this.targetDetector = targetDetector;
    }

    public void initRun() {

        lastDetection = null;
        inShootingZone = true;
    }

    public Command initRunCommand() {
        return Commands.runOnce(() -> {
            initRun();
        });
    }

    void setInShootingZone() {
        inShootingZone = true;
    }

    Command signalInShootingZoneCommand() {
        return Commands.runOnce(() -> {
            setInShootingZone();
        });
    }

    public boolean pollTargeting() {
        Optional<TargetDetectorInterface.Detection> detectionResult = targetDetector.getClosestVisibleTarget();
        if (detectionResult.isPresent()) {

            lastDetection = detectionResult.get();

            System.out.println("Detected! " + lastDetection);
            return true;
        } else {
            return false;
        }

    }

    // This target detector checks the limelight. If it sees the target, it returns that
    // if FireCOntrol has seen a target, it just returns that
    // Otherwise it returns a preset
    public class FireControlEndDetector implements TargetDetectorInterface {
        double presetTy;

        FireControlEndDetector(double presetTy) {
            this.presetTy = presetTy;
        }

        @Override
        public Optional<Detection> getClosestVisibleTarget() {
            var limelightResult = targetDetector.getClosestVisibleTarget();
            if (limelightResult.isEmpty()) {
                if (lastDetection == null) {
                    return Optional.of(new Detection(0, presetTy, 0));
                } else {
                    return Optional.of(lastDetection);
                }

            } else {
                return limelightResult;

            }
        }
    }

    public Command aimAndFireCommand(double presetTy) {
        TargetDetectorInterface targetDetector = new FireControlEndDetector(presetTy);
        return new LockOnOpenLoopCommand(targetDetector);
    }
    // public void fireAndReset() {

    // // Step 1. Check again, do I see the target - Yes, fire at the target?
    // // Step 2. If not, do I have the last target I saved - Yes, fire at the last target?
    // // Step

    // Command cmd = new ConditionalCommand(AutoAimAndFire,
    // FirePreset, null)
    // }

    // This is the command that tracks while the shooter run is occurring.
    public Command trackRunCommand() {
        return new WaitUntilCommand(() -> {
            return pollTargeting();
        });
    }
}
