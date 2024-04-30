package com.spartronics4915.frc2024.commands.advancedAutos;

import static edu.wpi.first.units.Units.Seconds;

import com.spartronics4915.frc2024.commands.AutoComponents;
import com.spartronics4915.frc2024.commands.LimelightAuto;
import com.spartronics4915.frc2024.commands.StationaryAutoAimCommand;
import com.spartronics4915.frc2024.commands.StationaryAutoAimVisionPose;
import com.spartronics4915.frc2024.commands.TableAutoAimCommand;
import com.spartronics4915.frc2024.commands.advancedAutos.AdvAutoStates.AutoStates;
import com.spartronics4915.frc2024.commands.advancedAutos.AdvAutoStates.NotePresence;
import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Set;


public class AdvAutoLogic {
    //TODO test scan and shoot logic
    //TODO test search and gather logic
    //TODO test cases where notes aren't present in robot or can't find another note, etc
    
    private static Measure<Time> autoTimeout = Seconds.of(1); 
    private static Measure<Time> searchTimeout = Seconds.of(2.5); 
    private static Measure<Time> aimDebounce = Seconds.of(0.1); //time to confirm the shotoer and swerve is on target
    private static Measure<Time> aimTimeout = Seconds.of(1); 

    public enum FieldPosition{
        ;

        private Translation2d position;
        private FieldPosition(Translation2d position) {
            this.position = position;
        }
    }

    private static VisionSubsystem mVision = VisionSubsystem.getInstance();
    private static SwerveDrive mSwerve = SwerveDrive.getInstance();
    private static ShooterWrist mShooterWrist = ShooterWrist.getInstance();



    private static Command setAutoStateCommand(AutoStates newState){
        return Commands.runOnce(() -> {AdvAutoStates.AutoState = newState;});
    }


    //Intake logic

    public static Command searchAndGather(){
        return Commands.either(
            gather(),
            Commands.defer(() -> searchForNote().until(
                mVision.getNoteLocator().getClosestVisibleTarget()::isPresent
            ).withTimeout(autoTimeout.in(Seconds)), Set.of())
            .andThen(gather()),
            () -> {
                var detection = mVision.getNoteLocator().getClosestVisibleTarget();

                return detection.isPresent();
            }
        );
    }

    public static Command searchForNote(){
        return setAutoStateCommand(AutoStates.SEARCH).andThen(
            Commands.run(
                //TODO add preference of direction based on where on the field the robot is
                () -> {
                    mSwerve.drive(new ChassisSpeeds(0, 0, Math.toRadians(360)), false);
                }
            )
        );
    }

    private static Command gather(){
        return Commands.parallel(
            setAutoStateCommand(AutoStates.GATHER),
            AutoComponents.groundIntake(),
            LimelightAuto.driveToNote()
        ).until(() -> {
            return AdvAutoStates.NotePresenceState == NotePresence.INTAKE;
        }).withTimeout(searchTimeout.in(Seconds));
    }


    //Shoot logic

    public static Command visionAimAndShoot(){
        return Commands.sequence(
            //loads into shooter
            new ConditionalCommand(
                Commands.none(),
                AutoComponents.loadIntoShooter(),
                () -> {
                    return AdvAutoStates.NotePresenceState == NotePresence.LOADED;
                }
            ),
            AutoComponents.warmUpShooter(),
            setAutoStateCommand(AutoStates.SCAN),
            //aims if the condition is satisfied, otherwise run scan command
            new ConditionalCommand(
                aimVision(),
                scanVision().until(
                    () -> mVision.getSpeakerTagLocator().getClosestVisibleTarget().isPresent()
                ).andThen(
                    aimVision()
                ),
                () -> {
                    var detection = mVision.getSpeakerTagLocator().getClosestVisibleTarget();

                    return detection.isPresent();
                }
            ).until(new Trigger(() -> {
                return AdvAutoStates.AutoState == AutoStates.AIM && mShooterWrist.atTarget() && mSwerve.atTarget();
            }).debounce(aimDebounce.in(Seconds)))
            .withTimeout(aimTimeout.in(Seconds)),
            Commands.print("I just tried to shoot a note!") //TODO test this and then remove
            // AutoComponents.shootFromLoaded()
        );
    }

    private static Command aimVision(){
        return Commands.parallel(
                setAutoStateCommand(AutoStates.AIM),
                new TableAutoAimCommand(),
                Commands.defer(() -> {
                    final var alliance = DriverStation.getAlliance().get();
                    final var speaker = alliance == Alliance.Blue
                            ? AutoComponents.BLUE_SPEAKER
                            : AutoComponents.RED_SPEAKER;

                    return StationaryAutoAimVisionPose.getStationaryAutoAimVisionOrPose(mVision.getSpeakerTagLocator(), speaker);
                }, Set.of())
            );
    }

    private static Command scanVision(){
        return Commands.parallel(
            setAutoStateCommand(AutoStates.SCAN),
            Commands.defer(() -> {
                final var alliance = DriverStation.getAlliance().get();
                final var speaker = alliance == Alliance.Blue
                        ? AutoComponents.BLUE_SPEAKER
                        : AutoComponents.RED_SPEAKER;

                return new StationaryAutoAimCommand(speaker);
            }, Set.of())
        );
    }

}
