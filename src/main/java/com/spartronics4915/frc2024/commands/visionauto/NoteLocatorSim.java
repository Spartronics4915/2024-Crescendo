package com.spartronics4915.frc2024.commands.visionauto;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;

import javax.swing.text.html.Option;

import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class NoteLocatorSim implements NoteLocatorInterface {

    SwerveDrive swerveDrive;
    final static ArrayList<Translation2d> noteLocations = new ArrayList<>(List.of(new Translation2d(2.9, 7)));

    public NoteLocatorSim(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    public Optional<NoteDetection> getClosestVisibleNote() {

        double minDist = 1e6;
        Pose2d currPose = swerveDrive.getPose();
        Translation2d currPosition = currPose.getTranslation();

        final double MAX_DEGREES = 27;
        final double ROBOT_HEIGHT = 0.3; // Meters
        final double VERT_TOP_VISIBILITY_THRESH = -20; // Degrees
        final double VERT_BOT_VISIBILITY_THRESH = -40; // Degrees
        Optional<NoteDetection> bestNote = Optional.empty();
        for (Translation2d currNoteLoc : noteLocations) {
            Translation2d botNoteVec = currNoteLoc.minus(currPosition);
            Rotation2d botNoteAngle = botNoteVec.getAngle();
            Rotation2d viewCenterNoteAngle = currPose.getRotation().minus(botNoteAngle);
            Rotation2d vertAngle = new Rotation2d(botNoteVec.getNorm(), -ROBOT_HEIGHT);
            double dist = botNoteVec.getNorm();
            if (dist < minDist) {

                bestNote = Optional
                        .of(new NoteDetection(viewCenterNoteAngle.getDegrees(), vertAngle.getDegrees(), dist));
            }
            // if(Math.abs(viewCenterNoteAngle.getDegrees())> MAX_DEGREES) {
            // return Optional.empty();
            // }

            double vertAngleDegrees = vertAngle.getDegrees();
            if((vertAngleDegrees < VERT_BOT_VISIBILITY_THRESH)) {
            return Optional.empty();
            }
            // System.out.println("view center: " + viewCenterNoteAngle.getDegrees() + " " + "vert: "
            //         + vertAngle.getDegrees() + " " + currPosition + " " + currNoteLoc);

        }

        return bestNote;
    }

    public double getTy() {
        return 10;

    }

    // Finds the angle from the center of the closest note. Will filter over 27 degress
    public OptionalDouble getTx() {
        final double MAX_ANGLE_DEGREES = 27;

        return OptionalDouble.of(10);

    }

}
