package com.spartronics4915.frc2024.commands;

import com.spartronics4915.frc2024.subsystems.Shooter;
import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

import static com.spartronics4915.frc2024.Constants.AutoAimConstants.kShooterHeight;
import static com.spartronics4915.frc2024.util.AutoAimFunctions.*;

public class MovingAutoAimCommand extends Command{
    private static final boolean kDebug = true;

    public SwerveDrive mSwerve;
    public ShooterWrist mShooterWrist;
    public boolean started = false;;
    public final Translation3d kTarget;

    static{
        //CHECKUP "warms up" the function, first run of this function takes roughly 20 ms
        movingAutoAim(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
            new ChassisSpeeds(0, 0, 0), 
            new Translation3d(0, 0, 0)
        );
    }

    //moving auto aim logging

    private static final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("MAADebug");

    private static final StructPublisher<Pose2d> kPoseInputPublisher = loggingTable.getStructTopic("PoseInput", Pose2d.struct).publish();
    private static final StructPublisher<Pose3d> kTargetInputPublisher = loggingTable.getStructTopic("TargetInput", Pose3d.struct).publish();
    private static final StructPublisher<Pose2d> kVelocityPublisher = loggingTable.getStructTopic("RobotVelocity", Pose2d.struct).publish();
    /*
     * RR = robot relative
     * FR = field relative
     */
    private static final StructPublisher<Pose3d> kTargetOutputRRPublisher = loggingTable.getStructTopic("TargetRROutput", Pose3d.struct).publish();
    private static final StructPublisher<Pose3d> kTargetOutputFRPublisher = loggingTable.getStructTopic("TargetFROutput", Pose3d.struct).publish();

    private static final StructPublisher<Rotation2d> kWristAnglePublisher = loggingTable.getStructTopic("Wrist-Angle-Output", Rotation2d.struct).publish();
    private static final StructPublisher<Rotation2d> kSwerveAnglePublisher = loggingTable.getStructTopic("Swerve-Angle-Output", Rotation2d.struct).publish();



    public MovingAutoAimCommand(Translation3d mTarget) {
        this.mSwerve = SwerveDrive.getInstance();
        this.mShooterWrist = ShooterWrist.getInstance();
        this.kTarget = mTarget;
        addRequirements(mShooterWrist);
    }

    StructPublisher<Pose3d> targetPublisher = NetworkTableInstance.getDefault().getTable("simStuff").getStructTopic("MAA", Pose3d.struct).publish();
    
    @Override
    public void initialize() {
        if (mSwerve.rotationIsDecoupled()) {
            end(true);
        } else {
            started = true;
            mSwerve.decoupleRotation();
        }
        super.initialize();
    }
    
    @Override
    public void execute() {
        var pose = mSwerve.getPose();
        var speeds = mSwerve.getFieldRelativeSpeeds(); 
        var aimingPoint = movingAutoAim(
            pose, 
            speeds,
            kTarget
        );
        if (aimingPoint.isEmpty()) {
            return;
        }

        if (kDebug) {
            kVelocityPublisher.accept(new Pose2d(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), new Rotation2d()));
            kTargetInputPublisher.accept(new Pose3d(kTarget, new Rotation3d()));
            kPoseInputPublisher.accept(pose);
        }

        var targetPos = aimingPoint.get() ;
        var shooterAngle = getShooterAngle(targetPos);
        mShooterWrist.publicSetRotationSetPoint(shooterAngle);

        targetPublisher.accept(new Pose3d(targetPos.plus(new Translation3d(
            mSwerve.getPose().getTranslation().getX(),
            mSwerve.getPose().getTranslation().getY(),
            0.0
        )), new Rotation3d()));

        var botAngle = getChassisAngle(targetPos); //base rotations 0 --> 360 //reverses the direction (ie now the back is facing the target)

        mSwerve.setDesiredAngle(botAngle); //error of ~15 degres when moving
        // System.out.println(mSwerve.getAngle().getDegrees());
        // System.out.println(botAngle.getDegrees());
        // System.out.println((mSwerve.getAngle().getDegrees() % 360 - botAngle.getDegrees() % 360));
        // System.out.println("next");

        if (kDebug) {
            kTargetOutputRRPublisher.accept(new Pose3d(targetPos, new Rotation3d()));
            kTargetOutputFRPublisher.accept(new Pose3d(targetPos.plus(new Translation3d(
                pose.getX(),
                pose.getY(),
                kShooterHeight
            )), new Rotation3d()));
            kWristAnglePublisher.accept(shooterAngle);
            kSwerveAnglePublisher.accept(botAngle);
        }
        
        super.execute();
    }

    public boolean atTarget(){
        return
            mSwerve.atTarget() &&
            mShooterWrist.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        if(started) mSwerve.recoupleRotation();
        super.end(interrupted);
    }
}
