package com.spartronics4915.frc2024.commands.advancedAutos;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2024.subsystems.Shooter;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake.IntakeState;
import com.spartronics4915.frc2024.subsystems.Shooter.ConveyorState;
import com.spartronics4915.frc2024.subsystems.Shooter.ShooterState;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;
import com.spartronics4915.frc2024.util.NoteVisualizer;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AdvAutoStates {
    public static final Measure<Time> kNotePresenceNoiseDebounce = Seconds.of(0.05);
    public static final Measure<Time> kNotePresenceDigestThreshold= Seconds.of(1);


    public static enum AutoStates{
        SHOOT,
        HUNT,
        SCAN, //searching for target
        AIM, //aiming at target
        SEARCH, //searching for note
        GATHER //gathering note
    }

    public static enum NotePresence{
        INTAKE,
        LOADED,
        OUTSIDE
    }
    public static NotePresence NotePresenceState = NotePresence.LOADED;
    public static AutoStates AutoState = AutoStates.SCAN;


    private static void setState(NotePresence state){
        NotePresenceState = state;
    }

    static{
        var mIntake = Intake.getInstance();
        var mShooter = Shooter.getInstance();

        if (RobotBase.isReal()) {
            var intakeBeamBreak = new Trigger(mIntake::beamBreakIsTriggered).debounce(kNotePresenceNoiseDebounce.in(Seconds));
            var shooterBeamBreak = new Trigger(mShooter::beamBreakIsTriggered).debounce(kNotePresenceNoiseDebounce.in(Seconds));
    
            intakeBeamBreak.onTrue(Commands.runOnce(() -> setState(NotePresence.INTAKE)).ignoringDisable(true));
            shooterBeamBreak.onTrue(Commands.runOnce(() -> setState(NotePresence.LOADED)).ignoringDisable(true));
            shooterBeamBreak.onFalse(Commands.runOnce(() -> setState(NotePresence.OUTSIDE)).ignoringDisable(true));
            
            intakeBeamBreak.negate().and(shooterBeamBreak.negate()).debounce(kNotePresenceDigestThreshold.in(Seconds))
                .onTrue(Commands.runOnce(() -> setState(NotePresence.OUTSIDE)).ignoringDisable(true));
            
        }

        if (RobotBase.isSimulation()) {
            initalizeSimTriggers();
        }
    }
    

    //logging stuff
    private static StringPublisher NotePresencePublisher = NetworkTableInstance.getDefault().getTable("Logging").getStringTopic("NotePresence").publish();
    private static StringPublisher AutoStatePublisher = NetworkTableInstance.getDefault().getTable("Logging").getStringTopic("AutoState").publish();

    public static void updateStateLogs(){
        NotePresencePublisher.accept(NotePresenceState.name());
        AutoStatePublisher.accept(AutoState.name());
    }

    private static void initalizeSimTriggers(){
        if (RobotBase.isReal()) {
            return;
        }

        var mIntake = Intake.getInstance();
        var mShooter = Shooter.getInstance();
        var mSwerve = SwerveDrive.getInstance();

        new Trigger(() -> {
            return (
                NotePresenceState == NotePresence.INTAKE && 
                mIntake.getState() == IntakeState.LOAD  && 
                mShooter.getConveyorState() == ConveyorState.IN
            );
        }).onTrue(
            Commands.runOnce(() -> setState(NotePresence.LOADED))
        );

        new Trigger(() -> {
            return (
                NotePresenceState == NotePresence.INTAKE && 
                mIntake.getState() == IntakeState.OUT 
            );
        }).onTrue(
            Commands.runOnce(() -> setState(NotePresence.OUTSIDE))
        );

        new Trigger(() -> {
            return (
                NotePresenceState == NotePresence.LOADED && 
                mShooter.getShooterState() == ShooterState.ON &&
                mShooter.getConveyorState() == ConveyorState.SHOOTING
            );
        }).onTrue(Commands.parallel(
            NoteVisualizer.visualizeTrajectoryCommand(),
            Commands.runOnce(() -> setState(NotePresence.OUTSIDE))
        ));

        ArrayList<Translation2d> noteLocations = new ArrayList<>(List.of(new Translation2d(2.9, 7),
            new Translation2d(2.9, 5.5), new Translation2d(2.9, 4.1)));

        
        new Trigger(() -> {
            var pose = mSwerve.getPose();
            boolean onTopOfNote = false;
            for (Translation2d note : noteLocations) {
                onTopOfNote |= note.getDistance(pose.getTranslation()) < 0.1;
            }
            return onTopOfNote && mIntake.getState() == IntakeState.IN && NotePresenceState == NotePresence.OUTSIDE;
        }).onTrue(
            Commands.runOnce(() -> {
                setState(NotePresence.INTAKE);
                mIntake.setState(IntakeState.OFF);
            })
        );
    }
}
