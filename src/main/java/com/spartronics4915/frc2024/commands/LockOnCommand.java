package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.Constants.BlingModes;
import com.spartronics4915.frc2024.Constants.Vision.VisionPipelines;
import com.spartronics4915.frc2024.commands.visionauto.NoteLocatorInterface;
import com.spartronics4915.frc2024.commands.visionauto.NoteLocatorLimeLight;
import com.spartronics4915.frc2024.commands.visionauto.NoteLocatorSim;
import com.spartronics4915.frc2024.commands.visionauto.TargetDetectorInterface;
import com.spartronics4915.frc2024.subsystems.Bling;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;
import com.spartronics4915.frc2024.subsystems.vision.LimelightDevice;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;
import java.util.Optional;
import com.spartronics4915.frc2024.commands.visionauto.NoteLocatorInterface.NoteDetection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class LockOnCommand extends Command {

    private final SwerveDrive mSwerve;
    private final VisionSubsystem mVision;
    private final LimelightDevice mLimelight;
    private final Bling mBling;
    private boolean cancellingEarly = false;

    private TargetDetectorInterface targetDetector;

    public LockOnCommand(TargetDetectorInterface targetDetector) {
        super();
        mSwerve = SwerveDrive.getInstance();
        mVision = VisionSubsystem.getInstance();
        mLimelight = mVision.getAlice();
        mBling = Bling.getInstance();

        this.targetDetector = targetDetector;
   
    }

    @Override
    public void initialize() {
        //do thing if rotation already decoupled
        if (mSwerve.rotationIsDecoupled()) {
            cancellingEarly = true;
            end(true);
        } else {
            cancellingEarly = false;
            mSwerve.decoupleRotation();
            // mLimelight.setVisionPipeline(VisionPipelines.DETECTOR_NOTE);
            mBling.setMode(BlingModes.SOLID);
            mBling.setColor(Color.kWhite);
        }
    }
    
    @Override
    public void execute() {

        Optional<TargetDetectorInterface.Detection> detectionResult = targetDetector.getClosestVisibleTarget();

        if (!detectionResult.isPresent()) {
            mBling.setColor(Color.kRed);
            return;
        }

        TargetDetectorInterface.Detection detection = detectionResult.get();
        
        Rotation2d swerveAngle = mSwerve.getAngle();
        double tx = detection.tx();
        Rotation2d limelightAngle = Rotation2d.fromDegrees(-tx);
        Rotation2d desiredAngle = Rotation2d.fromRotations(swerveAngle.getRotations() + limelightAngle.getRotations());
        mSwerve.setDesiredAngle(desiredAngle);
        // System.out.println("LOCKON:\nswerve: " + swerveAngle + "\nlimelight: " + limelightAngle + "\ndesired: " + desiredAngle);

        // This is the old logic before noteLocator
        // if (!loaded) mBling.setColor(Color.kWhite);
        // else if (loaded && mLimelight.getTv()) {
        //     mSwerve.setDesiredAngle(desiredAngle);
        //     double percent = MathUtil.clamp((30.0 - Math.abs(tx)) / 30.0, 0.0, 1.0);
        //     Color color = Bling.mix(Color.kLime, Color.kOrange, percent);
        //     mBling.setColor(color);
        // } else mBling.setColor(Color.kRed);
    }

    @Override
    public void end(boolean interrupted) {
        if (!cancellingEarly) {
            mSwerve.recoupleRotation();
            // mLimelight.setVisionPipeline(VisionPipelines.FIDUCIALS_3D);
            mBling.setMode(BlingModes.OFF);
        }
    }
}
