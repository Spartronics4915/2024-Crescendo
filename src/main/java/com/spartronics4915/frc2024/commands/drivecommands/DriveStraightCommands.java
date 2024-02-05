package com.spartronics4915.frc2024.commands.drivecommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;

public class DriveStraightCommands {

    public static class DriveStraightFixedDistance extends Command {

        double mDistance;
        Rotation2d mHeading;
        SwerveDrive mSwerveSubsystem;
        double headingX, headingY;
        Pose2d initialPt;
        TrapezoidProfile.Constraints mConstraints;
        double mDistTraveled;
        TrapezoidProfile mProfile;
        TrapezoidProfile.State mGoalState;
        double modeledVelocity;

        boolean mFinished;

        /*
         * Drives straight in a direction, the heading is CCW+ with 0 driving straight
         * forward
         */
        public DriveStraightFixedDistance(SwerveDrive swerveSubsystem, Rotation2d heading, double distance,
                TrapezoidProfile.Constraints constraints) {
            mDistance = distance;
            mHeading = heading;
            mSwerveSubsystem = swerveSubsystem;
            mConstraints = constraints;
            addRequirements(mSwerveSubsystem);
            mProfile = new TrapezoidProfile(mConstraints);
            headingX = heading.getCos();
            headingY = heading.getSin();
            mGoalState = new TrapezoidProfile.State(distance, 0);
        }

        public void initialize() {
            initialPt = mSwerveSubsystem.getPose();
            modeledVelocity = 0;
        }

        public void execute() {

            final double dT = 1. / 50;
            Pose2d currPt = mSwerveSubsystem.getPose();
            mDistTraveled = currPt.getTranslation().minus(initialPt.getTranslation()).getNorm();
            TrapezoidProfile.State currState = new TrapezoidProfile.State(mDistTraveled, modeledVelocity);
            TrapezoidProfile.State newState = mProfile.calculate(dT, currState, mGoalState);
            modeledVelocity = newState.velocity;

            ChassisSpeeds newSpeed = new ChassisSpeeds(headingX * modeledVelocity, headingY * modeledVelocity, 0);
            mSwerveSubsystem.drive(newSpeed, false);

            if(Math.abs(newState.position - mGoalState.position) < 0.005) {
                mFinished = true;
            }

        }

        @Override
        public boolean isFinished() {

            return mFinished;
        }

    }

}
