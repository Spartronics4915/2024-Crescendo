package com.spartronics4915.frc2024.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

import static com.spartronics4915.frc2024.util.AutoAimFunctions.getChassisAngle;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;

/*
 * this autoAim is based on an interpolation table stored statically in this file
 */
public class TableAutoAimCommand extends Command {
    private record TableEntry(Measure<Distance> distance, Rotation2d shooterAngle) implements Comparable<TableEntry> {
        @Override
        public int compareTo(TableEntry o) {
            return this.distance.compareTo(o.distance);
        }
    }

    private static InterpolatingDoubleTreeMap mShooterAngleTable = new InterpolatingDoubleTreeMap();
    private static LinkedList<TableEntry> data = new LinkedList<>();

    static {
        data.add(new TableEntry(Units.Meters.of(10), Rotation2d.fromDegrees(10)));
        var x = new InterpolatingDoubleTreeMap();

        for (TableEntry tableEntry : data) {
            mShooterAngleTable.put(tableEntry.distance.in(Meters), tableEntry.shooterAngle.getRadians());
        }
    }

    private static Rotation2d getShooterAngle(Measure<Distance> distance) {
        return Rotation2d.fromRadians(mShooterAngleTable.get(distance.in(Meters)));
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

    private Measure<Distance> findDistance() {
        return Units.Meters.of(0.0);
    }

    private Rotation2d getBotAngle() {
        return new Rotation2d();
    }

    @Override
    public void execute() {

        Pose2d pose = mSwerve.getPose();

        mShooterWrist.publicSetRotationSetPoint(getShooterAngle(findDistance()));

        mSwerve.setDesiredAngle(
            getChassisAngle(kTarget.minus(new Translation3d(
                pose.getX(),
                pose.getY(),
                0.0)))
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (!cancellingEarly) {
            mSwerve.recoupleRotation();
        }
    }
}
