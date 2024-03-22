package com.spartronics4915.frc2024.commands.advancedAutos;

import static edu.wpi.first.units.Units.Seconds;

import com.spartronics4915.frc2024.commands.AutoComponents;
import com.spartronics4915.frc2024.commands.LimelightAuto;
import com.spartronics4915.frc2024.commands.advancedAutos.AdvAutoStates.NotePresence;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class AdvAutoLogic {
    private static Measure<Time> autoTimeout = Seconds.of(1); 

    public enum FieldPosition{
        ;

        private Translation2d position;
        private FieldPosition(Translation2d position) {
            this.position = position;
        }
    }

    private static VisionSubsystem mVision = VisionSubsystem.getInstance();
    private static SwerveDrive mSwerve = SwerveDrive.getInstance();



    //Intake logic

    public static Command searchAndGather(){
        return Commands.either(
            gather(),
            searchForNote().until(
                mVision.getNoteLocator().getClosestVisibleTarget()::isPresent
            ).withTimeout(autoTimeout.in(Seconds)).andThen(mSwerve.recoupleRotationCommand())
            .andThen(gather()),
            () -> {
                var detection = mVision.getNoteLocator().getClosestVisibleTarget();

                if (detection.isEmpty()) {
                    return false;
                }

                return detection.isPresent();
            }
        );
    }

    private static Command searchForNote(){
        return mSwerve.decoupleRotationCommand().andThen(
            Commands.run(
                //TODO add preference of direction based on where on the field the robot is
                () -> mSwerve.setDesiredAngle(
                    Rotation2d.fromDegrees(mSwerve.getAngle().getDegrees() + (360 / (autoTimeout.in(Seconds)/50)))
                    //do one full rotation within the timeout
                )
            )
        );
    }

    private static Command gather(){
        return Commands.parallel(
            AutoComponents.groundIntake(),
            LimelightAuto.driveToNote()
        ).until(() -> {
            return AdvAutoStates.NotePresenceState == NotePresence.INTAKE;
        }).withTimeout(autoTimeout.in(Seconds));
    }

}
