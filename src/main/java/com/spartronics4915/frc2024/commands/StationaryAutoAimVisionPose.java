package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.commands.visionauto.LockOnOpenLoopCommand;
import com.spartronics4915.frc2024.commands.visionauto.TargetDetectorInterface;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class StationaryAutoAimVisionPose {

    public static Command getStationaryAutoAimVisionOrPose(
            TargetDetectorInterface speakerTargetDetector,
            Translation3d mTarget) {

        Command useSpeakerTagCommand = new AlignToSpeakerCommand(); // new LockOnOpenLoopCommand(speakerTargetDetector);
        Command usePoseCommand = new StationaryAutoAimCommand(mTarget);
        return useSpeakerTagCommand;
        // return Commands.either(useSpeakerTagCommand,
        //         usePoseCommand,
        //         () -> {
        //             return speakerTargetDetector.getClosestVisibleTarget().isPresent();
        //         });
    }
}
