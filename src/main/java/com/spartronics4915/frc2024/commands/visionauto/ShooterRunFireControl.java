package com.spartronics4915.frc2024.commands.visionauto;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.Map;
import java.util.Optional;

public class ShooterRunFireControl extends SubsystemBase {

    TargetDetectorInterface.Detection lastDetection;
    boolean inShootingZone;
    TargetDetectorInterface targetDetector;
    GenericEntry inputTyEntry, inputTxEntry;
    GenericEntry outputTyEntry;
    GenericEntry detectionEntry;

    public ShooterRunFireControl(TargetDetectorInterface targetDetector) {

        this.targetDetector = targetDetector;
        initShuffleboard();

    }

    public void initShuffleboard() {
        ShuffleboardLayout mShuffleBoardWidget = Shuffleboard
                .getTab("Shooter")
                .getLayout("FireControl", BuiltInLayouts.kList)
                .withSize(3, 3).withProperties(Map.of("Label position", "TOP"));
        inputTyEntry = mShuffleBoardWidget.add("InputTy", 0).getEntry();
        inputTxEntry = mShuffleBoardWidget.add("InputTx", 0).getEntry();
        outputTyEntry = mShuffleBoardWidget.add("OutputTy", 0).getEntry();
        detectionEntry = mShuffleBoardWidget.add("Detection", false).getEntry();

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
            if (detectionResult.get().ty() < 0) {
                return false;
            }

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

    @Override
    public void periodic() {

        var detectionResult = targetDetector.getClosestVisibleTarget();
        if(detectionResult.isEmpty()) {
            inputTyEntry.setBoolean(false);
            outputTyEntry.setBoolean(false);
            inputTxEntry.setBoolean(false);
            detectionEntry.setBoolean(false);
        } else
        {
            double inputTy = detectionResult.get().ty();
            double outputTy = LinearAutoAim.computeOutputAngle(inputTy);

            inputTyEntry.setDouble(inputTy);
            outputTyEntry.setDouble(outputTy);
            inputTxEntry.setDouble(detectionResult.get().tx());
            detectionEntry.setBoolean(true);

        }

    }
}
