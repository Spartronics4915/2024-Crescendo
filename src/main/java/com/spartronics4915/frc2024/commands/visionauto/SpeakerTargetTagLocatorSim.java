package com.spartronics4915.frc2024.commands.visionauto;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;

import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SpeakerTargetTagLocatorSim implements TargetDetectorInterface {

    SwerveDrive swerveDrive;

    final static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    Translation3d targetPosition;

    public SpeakerTargetTagLocatorSim(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    public Optional<Detection> getClosestVisibleTarget() {

        // Needs to be here for sim in case the alliance changes in sim.

        if (DriverStation.getAlliance().get() == Alliance.Blue) {

            targetPosition = fieldLayout.getTagPose(7).get().getTranslation();
        } else {
            targetPosition = fieldLayout.getTagPose(4).get().getTranslation();

        }
        double minDist = 1e6;
        Pose2d currPose = swerveDrive.getPose();
        Rotation2d currRobotView = currPose.getRotation();
        final double CAMERA_HEIGHT = 0.5;
        Translation3d currPosition = new Translation3d(currPose.getTranslation().getX(),
                currPose.getTranslation().getY(), CAMERA_HEIGHT);

        final double MAX_DEGREES = 45;
        final double ROBOT_HEIGHT = 0.3; // Meters
        final double VERT_BOT_VISIBILITY_THRESH = 80; // Degrees
        Translation3d botTargetVec = targetPosition.minus(currPosition);
        Translation2d botTargetFloorVec = new Translation2d(botTargetVec.getX(), botTargetVec.getY());

        Rotation2d botTargetVertAngle = new Translation2d(botTargetFloorVec.getNorm(), botTargetVec.getZ()).getAngle();
        Rotation2d viewCenterNoteAngle = botTargetFloorVec.getAngle().minus(currRobotView);

        if (Math.abs(viewCenterNoteAngle.getDegrees()) > MAX_DEGREES) {
            return Optional.empty();
        }

        double vertAngleDegrees = botTargetVertAngle.getDegrees();
        if ((vertAngleDegrees > VERT_BOT_VISIBILITY_THRESH)) {
            return Optional.empty();
        }

        double dist = botTargetVec.getZ() / botTargetVertAngle.getTan();

        return Optional.of(new Detection(-viewCenterNoteAngle.getDegrees(), vertAngleDegrees, dist));
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
