package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.Constants.BlingModes;
import com.spartronics4915.frc2024.Constants.Vision.VisionPipelines;
import com.spartronics4915.frc2024.subsystems.Bling;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;
import com.spartronics4915.frc2024.subsystems.vision.LimelightDevice;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToSpeakerCommand extends Command {

    private final SwerveDrive mSwerve;
    private final VisionSubsystem mVision;
    private final LimelightDevice mLimelight;
    private final Bling mBling;
    private int priorityID;
    private boolean cancellingEarly;
    private boolean cantSeeTag;
    private boolean triggeredAlign;
    private Rotation2d desiredRotation;

    public AlignToSpeakerCommand() {
        super();
        mSwerve = SwerveDrive.getInstance();
        mVision = VisionSubsystem.getInstance();
        mLimelight = mVision.getBob();
        mBling = Bling.getInstance();
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
            var alliance = DriverStation.getAlliance();
            if (alliance.isEmpty()) {
                System.out.println("Ending AlignToSpeakerCommand as there is no alliance set.");
                cancellingEarly = true;
            } else if (alliance.get().equals(Alliance.Blue)) {
                priorityID = 7;
            } else if (alliance.get().equals(Alliance.Red)) {
                priorityID = 4;
            } else {
                System.out.println("Error with AlignToSpeakerCommand! Failed to get alliance data!");
                cancellingEarly = true;
            }
            if (!cancellingEarly) {
            mSwerve.decoupleRotation();
            mBling.setMode(BlingModes.SOLID);
            mLimelight.setPriorityTagID(priorityID);
            }
        }
    }
    
    @Override
    public void execute() {
        if (mLimelight.getTv() && !cancellingEarly) {
        Rotation2d swerveAngle = mSwerve.getAngle();
        double tx = mLimelight.getTx();
        Rotation2d limelightAngle = Rotation2d.fromDegrees(-tx);
        Rotation2d desiredAngle = Rotation2d.fromRotations(swerveAngle.getRotations() + limelightAngle.getRotations());
            if (!triggeredAlign) {
                mSwerve.setDesiredAngle(desiredAngle);
                desiredRotation = desiredAngle;
                triggeredAlign = true;
            }
            double percent = MathUtil.clamp((30.0 - Math.abs(tx)) / 30.0, 0.0, 1.0);
            Color color = Bling.mix(Color.kLime, Color.kOrange, percent);
            mBling.setColor(color);
        } else if (!cancellingEarly && !cantSeeTag) {
            cantSeeTag = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (cancellingEarly || cantSeeTag) return true;
        return mLimelight.getTv() && triggeredAlign && (Math.abs(mSwerve.getAngle().minus(desiredRotation).getDegrees()) < 1.5);
    }

    @Override
    public void end(boolean interrupted) {
        if (!cancellingEarly) {
            mSwerve.recoupleRotation();
            mBling.setMode(BlingModes.OFF);
            mLimelight.resetPriorityTagID();
        }
    }
}
