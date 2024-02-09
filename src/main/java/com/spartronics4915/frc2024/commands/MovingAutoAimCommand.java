package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.subsystems.Shooter;
import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import static com.spartronics4915.frc2024.util.AutoAimFunctions.*;

public class MovingAutoAimCommand extends Command{
    public SwerveDrive mSwerve;
    public ShooterWrist mShooterWrist;
    public final Translation3d kTarget;

    static{
        //CHECKUP "warms up" the function, first run of this function takes roughly 20 ms
        movingAutoAim(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
            new ChassisSpeeds(0, 0, 0), 
            new Translation3d(0, 0, 0)
        );
    }

    public MovingAutoAimCommand(Translation3d mTarget) {
        this.mSwerve = SwerveDrive.getInstance();
        this.mShooterWrist = ShooterWrist.getInstance();
        this.kTarget = mTarget;
        addRequirements(mSwerve, mShooterWrist);
    }

    
    @Override
    public void initialize() {
        mSwerve.decoupleRotation();
        super.initialize();
    }
    
    @Override
    public void execute() {
        var AimingPoint = movingAutoAim(
            mSwerve.getPose(), 
            mSwerve.getFieldRelativeSpeeds(), 
            kTarget
        );
        if (AimingPoint.isEmpty()) {
            return;
        }
        var angles = getShooterAngle(AimingPoint.get());
        mSwerve.setDesiredAngle(new Rotation2d(angles.getZ()));
        mShooterWrist.setRotationSetPoint(new Rotation2d(angles.getY()));
        super.execute();
    }

    public boolean atTarget(){
        return
            true &&//TODO implement swerve check
            mShooterWrist.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        mSwerve.recoupleRotation();
        super.end(interrupted);
    }
}
