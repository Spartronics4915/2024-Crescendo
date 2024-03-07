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

        data.add(new TableEntry(Rotation2d.fromDegrees(19.15) , Rotation2d.fromDegrees(46.25)));
        data.add(new TableEntry(Rotation2d.fromDegrees(9.85) , Rotation2d.fromDegrees(33.5)));
        data.add(new TableEntry(Rotation2d.fromDegrees(5.01) , Rotation2d.fromDegrees(27.5)));
        data.add(new TableEntry(Rotation2d.fromDegrees(1.7) , Rotation2d.fromDegrees(22.5)));
        data.add(new TableEntry(Rotation2d.fromDegrees(-1.13) , Rotation2d.fromDegrees(17.25)));


        for (TableEntry tableEntry : data) {
            mShooterAngleTable.put(tableEntry.ty.getDegrees(), tableEntry.shooterAngle.getRadians());
        }
    }

    private static Rotation2d getShooterAngle(Rotation2d ty) {
        return Rotation2d.fromRadians(mShooterAngleTable.get(ty.getDegrees()));
    };

    private Translation3d kTarget;

    private final SwerveDrive mSwerve = SwerveDrive.getInstance();
    private final ShooterWrist mShooterWrist = ShooterWrist.getInstance();

    private boolean cancellingEarly;

    public TableAutoAimCommand(Translation3d kTarget) {
        this.kTarget = kTarget;
    }

    @Override
    public void initialize() {
        // do thing if rotation already decoupled
        if (mSwerve.rotationIsDecoupled()) {
            cancellingEarly = true;
            end(true);
        } else {
            cancellingEarly = false;
            mSwerve.decoupleRotation();
        }
    }

    private Measure<Distance> findDistance(Pose2d chassisLocation) {
        return Units.Meters.of(kTarget.toTranslation2d().getDistance(chassisLocation.getTranslation()));
    }

    private Rotation2d getBotAngle(Pose2d chassisLocation) {
        return getChassisAngle(kTarget.minus(new Translation3d(
            chassisLocation.getX(),
            chassisLocation.getY(),
            0.0)
        ));
    }

    @Override
    public void execute() {

        Pose2d pose = mSwerve.getPose();

        mShooterWrist.publicSetRotationSetPoint(getShooterAngle(Rotation2d.fromDegrees(mVision.getBob().getTy())));

        // mSwerve.setDesiredAngle(getBotAngle(pose));
    }

    @Override
    public void end(boolean interrupted) {
        if (!cancellingEarly) {
            mSwerve.recoupleRotation();
        }
    }
}
