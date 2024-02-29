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
        System.out.println("AlignToSpeaker created");
        mSwerve = SwerveDrive.getInstance();
        mVision = VisionSubsystem.getInstance();
        mLimelight = mVision.getBob();
        mBling = Bling.getInstance();
    }

    @Override
    public void initialize() {
        System.out.println("AlignToSpeaker initialized");
        //do thing if rotation already decoupled
        if (mSwerve.rotationIsDecoupled()) {
            System.out.println("and we're ending early");
            cancellingEarly = true;
        } else {
            System.out.println("and we're NOT ending early");
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
        System.out.println("triggered align is " + triggeredAlign);
    }
    
    @Override
    public void execute() {
        if (mLimelight.getTv() && !cancellingEarly) {
        Rotation2d swerveAngle = mSwerve.getAngle();
        double tx = mLimelight.getTx();
        Rotation2d limelightAngle = Rotation2d.fromDegrees(-tx);
        Rotation2d desiredAngle = Rotation2d.fromRotations(swerveAngle.getRotations() + limelightAngle.getRotations());
        // System.out.println("ALIGN:\nswerve: " + swerveAngle + "\nlimelight: " + limelightAngle + "\ndesired: " + desiredAngle);
            if (!triggeredAlign) {
                mSwerve.setDesiredAngle(desiredAngle);
                desiredRotation = desiredAngle;
                triggeredAlign = true;
                System.out.println("Well, this is it.\nOur desired angle is " + desiredAngle + ", and the current rotation from the swerve drive is " + swerveAngle + ".\nThe angle reading from the limelight is " + tx + ", which becomes a rotation of " + limelightAngle + ".\nWe're aligning to " + priorityID + " and tv is " + mLimelight.getTv() + ".\nLet's do this!");
            }
            double percent = MathUtil.clamp((30.0 - Math.abs(tx)) / 30.0, 0.0, 1.0);
            Color color = Bling.mix(Color.kLime, Color.kOrange, percent);
            mBling.setColor(color);
        } else if (!cancellingEarly && !cantSeeTag) {
            System.out.println("Can't see the tag! Ending AlignToSpeaker.");
            cantSeeTag = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (cancellingEarly || cantSeeTag) return true;
        System.out.println(cancellingEarly + ", " + mLimelight.getTv() + ", " + triggeredAlign + ", " + Math.abs(mSwerve.getAngle().minus(desiredRotation).getDegrees()));
        return mLimelight.getTv() && triggeredAlign && (Math.abs(mSwerve.getAngle().minus(desiredRotation).getDegrees()) < 1.5);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDING ALIGN!!! interrupted=" + interrupted);
        if (!cancellingEarly) {
            mSwerve.recoupleRotation();
            mBling.setMode(BlingModes.OFF);
            mLimelight.resetPriorityTagID();
        }
    }
}
