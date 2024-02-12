package com.spartronics4915.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

import static com.spartronics4915.frc2024.Constants.AutoAimConstants.*;

import java.util.function.*;

import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;

public class NoteVisualizer {
    private static Translation3d getAtTime(Translation3d posI, Translation3d velI, Translation3d accel, double seconds){
        return posI.plus(velI.times(seconds)).plus(accel.times(seconds*seconds).times(0.5));
    }

    private static Translation3d nullPosition = new Translation3d(0, 0, -10); 

    public final Translation3d initalPos;
    public final Translation3d initialVel;
    public final Translation3d accel = new Translation3d(0, 0, kGravity);

    Predicate<Translation3d> isValidPos;
    
    public NoteVisualizer(final Translation3d aimingPoint, final Pose2d robotPos) {
        super();

        initalPos = new Translation3d(
            robotPos.getX(),
            robotPos.getY(),
            kShooterHeight
        );

        
        initialVel = aimingPoint.div(aimingPoint.getNorm()).times(kShooterSpeed);
        System.out.println(initialVel);
    }

    public static NoteVisualizer shoot(){
        var swerve = SwerveDrive.getInstance();

        var x = new Rotation3d(
            0.0,
            Rotation2d.fromRotations(ShooterWrist.getInstance().getSetPoint().position).getRadians(), 
            swerve.getAngle().getRadians()
        );

        System.out.println(x.getZ());
        System.out.println(x.getY());
        double aimX = Math.cos(x.getZ());
        double aimY = Math.sin(x.getZ());
        double aimZ = Math.sin(x.getY()); //vertical value

        var outputAngle = new Translation3d(
            Double.isNaN(aimX) ? 0.0 : aimX, 
            Double.isNaN(aimY) ? 0.0 : aimY, 
            Double.isNaN(aimZ) ? 0.0 : aimZ
        );

        outputAngle = outputAngle.div(outputAngle.getNorm()).times(kShooterSpeed).plus(new Translation3d(
            swerve.getFieldRelativeSpeeds().vxMetersPerSecond, 
            swerve.getFieldRelativeSpeeds().vyMetersPerSecond, 
        0.0));

        System.out.println(outputAngle);

        return new NoteVisualizer(
            outputAngle,
            SwerveDrive.getInstance().getPose()
        );
    }

    static StructPublisher<Pose3d> notePublisher = NetworkTableInstance.getDefault().getTable("simStuff").getStructTopic("noteVis", Pose3d.struct).publish();

    private static void publishPos(Translation3d pos){
        notePublisher.accept(new Pose3d(pos, new Rotation3d(0, 0, 0)));
    }


    public Command visualizeTrajectory(){
        final Timer timer = new Timer();
        timer.start(); 
        return Commands.deadline(Commands.waitUntil(() -> getAtTime(initalPos, initialVel, accel, timer.get()).getZ() < 0), 
            Commands.runEnd(() -> {
                publishPos(getAtTime(initalPos, initialVel, accel, timer.get()));
            }, () -> {
                publishPos(nullPosition);
            })
        );
    }
}
