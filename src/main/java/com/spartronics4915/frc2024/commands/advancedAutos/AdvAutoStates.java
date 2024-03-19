package com.spartronics4915.frc2024.commands.advancedAutos;

import static edu.wpi.first.units.Units.Seconds;

import com.spartronics4915.frc2024.subsystems.Shooter;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AdvAutoStates {
    public static final Measure<Time> kNotePresenceNoiseDebounce = Seconds.of(0.05);
    public static final Measure<Time> kNotePresenceDigestThreshold= Seconds.of(0.05);


    public static enum AutoStates{
        SHOOT,
        HUNT,
        SCAN,
        AIM,
        SEARCH,
        GATHER
    }

    public static enum NotePresence{
        INTAKE,
        LOADED,
        OUTSIDE
    }
    public static NotePresence NotePresenceState = NotePresence.LOADED;
    public static AutoStates AutoState = AutoStates.AIM;


    private static void setState(NotePresence state){
        NotePresenceState = state;
    }

    static{
        var mIntake = Intake.getInstance();
        var mShooter = Shooter.getInstance();

        var intakeBeamBreak = new Trigger(mIntake::beamBreakIsTriggered).debounce(kNotePresenceNoiseDebounce.in(Seconds));
        var shooterBeamBreak = new Trigger(mShooter::beamBreakIsTriggered).debounce(kNotePresenceNoiseDebounce.in(Seconds));

        intakeBeamBreak.onTrue(Commands.run(() -> setState(NotePresence.INTAKE)));
        shooterBeamBreak.onTrue(Commands.run(() -> setState(NotePresence.LOADED)));
        shooterBeamBreak.onFalse(Commands.run(() -> setState(NotePresence.OUTSIDE)));
        
        intakeBeamBreak.debounce(kNotePresenceDigestThreshold.in(Seconds)).negate()
        .and(
            shooterBeamBreak.debounce(kNotePresenceDigestThreshold.in(Seconds)).negate()
        ).onTrue(Commands.run(() -> setState(NotePresence.OUTSIDE)));
        
    }
    

    //logging stuff
    private static StringPublisher NotePresencePublisher = NetworkTableInstance.getDefault().getTable("StateLogging").getStringTopic("NotePresence").publish();
    private static StringPublisher AutoStatePublisher = NetworkTableInstance.getDefault().getTable("StateLogging").getStringTopic("AutoState").publish();

    public static void updateStateLogs(){
        NotePresencePublisher.accept(NotePresenceState.name());
        AutoStatePublisher.accept(AutoState.name());
    }
}
