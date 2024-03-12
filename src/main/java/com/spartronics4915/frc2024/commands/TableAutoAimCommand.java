package com.spartronics4915.frc2024.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.LinkedList;

import com.spartronics4915.frc2024.Constants.BlingModes;
import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;

import static com.spartronics4915.frc2024.util.AutoAimFunctions.getChassisAngle;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;

/*
 * this autoAim is based on an interpolation table stored statically in this file
 */
public class TableAutoAimCommand extends Command {
    private record TableEntry(Rotation2d ty, Rotation2d shooterAngle) {
    }

    private static InterpolatingDoubleTreeMap mShooterAngleTable = new InterpolatingDoubleTreeMap();
    private static LinkedList<TableEntry> data = new LinkedList<>();
    private VisionSubsystem mVision = VisionSubsystem.getInstance();
    /*
     * add entries to the data list in this static block
     */
    static {
        // BE CAREFUL THERE IS A CORRECTION FACTOR BELOW BECAUSE THIS IS OLD DATA data for 
        data.add(new TableEntry(Rotation2d.fromDegrees(13.1) , Rotation2d.fromDegrees(50.3)));
        data.add(new TableEntry(Rotation2d.fromDegrees(5.6) , Rotation2d.fromDegrees(38.9)));
        data.add(new TableEntry(Rotation2d.fromDegrees(2.5) , Rotation2d.fromDegrees(32.9)));
        data.add(new TableEntry(Rotation2d.fromDegrees(0.69) , Rotation2d.fromDegrees(29.8)));
        data.add(new TableEntry(Rotation2d.fromDegrees(0.5) , Rotation2d.fromDegrees(29.6)));
        data.add(new TableEntry(Rotation2d.fromDegrees(-1.5) , Rotation2d.fromDegrees(25.1)));
        data.add(new TableEntry(Rotation2d.fromDegrees(-3.3) , Rotation2d.fromDegrees(23.2)));



        for (TableEntry tableEntry : data) {
            mShooterAngleTable.put(tableEntry.ty.getDegrees(), tableEntry.shooterAngle.getRadians());
        }
    }

    private static Rotation2d getShooterAngle(Rotation2d ty) {
        return Rotation2d.fromRadians(mShooterAngleTable.get(ty.getDegrees()));
    };


    private final ShooterWrist mShooterWrist = ShooterWrist.getInstance();


    public TableAutoAimCommand() {
        //TODO add Tag priority and alliance tag 
    }

    public boolean atTarget() {
        return mShooterWrist.atTarget();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        Rotation2d predictedAngle = getShooterAngle(Rotation2d.fromDegrees(mVision.getBob().getTy()));
        Rotation2d correctedAngle = predictedAngle.plus(Rotation2d.fromDegrees(2));

        mShooterWrist.publicSetRotationSetPoint(correctedAngle);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
