package com.spartronics4915.frc2024.commands.visionauto;

import com.spartronics4915.frc2024.Constants.BlingModes;
import com.spartronics4915.frc2024.subsystems.Bling;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;

public class LockOnOpenLoopCommand extends Command {

    private final SwerveDrive mSwerve;
    private final Bling mBling;
    private boolean cancellingEarly;
    private boolean cantSeeTag;
    private boolean triggeredAlign;
    private Rotation2d desiredRotation;
    TargetDetectorInterface targetDetector;

    public LockOnOpenLoopCommand(TargetDetectorInterface targetDetector) {
        super();
        mSwerve = SwerveDrive.getInstance();
        mBling = Bling.getInstance();
        this.targetDetector = targetDetector;
    }

    @Override
    public void initialize() {
        if (mSwerve.rotationIsDecoupled()) {
            cancellingEarly = true;
        } else {
            cancellingEarly = false;
            triggeredAlign = false;
            cantSeeTag = false;
            desiredRotation = new Rotation2d();
            if (!cancellingEarly) {
                mSwerve.decoupleRotation();
                mBling.setMode(BlingModes.SOLID);
                // mLimelight.setPriorityTagID(priorityID);
            }
        }
    }

    @Override
    public void execute() {

        if (cancellingEarly || cantSeeTag)
            return;

        Optional<TargetDetectorInterface.Detection> detectionResult = targetDetector.getClosestVisibleTarget();

        if (detectionResult.isEmpty()) {
            cantSeeTag = true;
            return;
        }

        if (triggeredAlign) {
            mSwerve.drive(new ChassisSpeeds(0,0,0), false);
            return;
        }

        TargetDetectorInterface.Detection detection = detectionResult.get();
        Rotation2d swerveAngle = mSwerve.getAngle();
        double tx = detection.tx();
        Rotation2d limelightAngle = Rotation2d.fromDegrees(-tx);
        Rotation2d desiredAngle = Rotation2d
                .fromRotations(swerveAngle.getRotations() + limelightAngle.getRotations());
        mSwerve.setDesiredAngle(desiredAngle);
        desiredRotation = desiredAngle;
        triggeredAlign = true;

    }

    @Override
    public boolean isFinished() {
        if (cancellingEarly || cantSeeTag)
            return true;

        return triggeredAlign
                && (Math.abs(mSwerve.getAngle().minus(desiredRotation).getDegrees()) < 1.5);
    }

    @Override
    public void end(boolean interrupted) {
        if (!cancellingEarly) {
            mSwerve.recoupleRotation();
        }
    }

}
