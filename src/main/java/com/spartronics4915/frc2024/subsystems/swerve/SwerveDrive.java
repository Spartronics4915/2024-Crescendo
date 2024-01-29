package com.spartronics4915.frc2024.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2024.Constants.Drive.*;

import java.util.List;
import java.util.stream.Stream;

public class SwerveDrive extends SubsystemBase {
    private static SwerveDrive mInstance;

    private final SwerveModule mFrontLeft;
    private final SwerveModule mBackLeft;
    private final SwerveModule mBackRight;
    private final SwerveModule mFrontRight;

    private final SwerveModule[] mModules;
    private final Translation2d[] mModuleLocations;

    private final Pigeon2 mIMU;

    private final PIDController mAngleController;
    private Rotation2d mDesiredAngle;
    private boolean mRotationIsIndependent;

    private final SwerveDriveKinematics mKinematics;
    private final SwerveDrivePoseEstimator mPoseEstimator;

    private SwerveDrive() {
        mFrontLeft = new SwerveModule(kFrontLeft);
        mBackLeft = new SwerveModule(kBackLeft);
        mBackRight = new SwerveModule(kBackRight);
        mFrontRight = new SwerveModule(kFrontRight);

        mModules = new SwerveModule[] { mFrontLeft, mBackLeft, mBackRight, mFrontRight };

        mIMU = new Pigeon2(kPigeon2ID);

        {
            final var pc = kAngleControllerPIDConstants;
            mAngleController = new PIDController(pc.p(), pc.i(), pc.d());
        }

        mRotationIsIndependent = false;

        mModuleLocations = (Translation2d[]) Stream.of(mModules).map((m) -> m.getLocation()).toArray();

        mKinematics = new SwerveDriveKinematics(mModuleLocations);

        {
            final var stateStdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1, 0.1);
            final var visionMeasurementStdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1, 0.1);

            // TODO: change initial pose estimate
            mPoseEstimator = new SwerveDrivePoseEstimator(mKinematics, getAngle(), getModulePositions(), new Pose2d(),
                    stateStdDevs, visionMeasurementStdDevs);
        }
    }

    public static SwerveDrive getInstance() {
        if (mInstance == null) {
            mInstance = new SwerveDrive();
        }
        return mInstance;
    }

    public void drive(ChassisSpeeds speeds, final boolean fieldRelative) {
        drive(speeds, fieldRelative, mRotationIsIndependent);
    }

    private void drive(final ChassisSpeeds speeds, final boolean fieldRelative, final boolean rotationIndependent) {
        final ChassisSpeeds _speeds;
        if (fieldRelative) {
            _speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
        } else {
            _speeds = speeds;
        }

        if (rotationIndependent) {
            _speeds.omegaRadiansPerSecond = mAngleController.calculate(getAngle().getRadians(), mDesiredAngle.getRadians());
        }

        final var moduleStates = mKinematics.toSwerveModuleStates(_speeds);

        final var moduleStatesIterator = List.of(moduleStates).iterator();

        for (SwerveModule m : mModules) {
            m.setDesiredState(moduleStatesIterator.next());
        }
    }

    public void decoupleRotation() {
        setDesiredAngle(getAngle());
        mRotationIsIndependent = true;
    }

    public Command decoupleRotationCommand() {
        return runOnce(this::decoupleRotation);
    }

    public void recoupleRotation() {
        mRotationIsIndependent = false;
    }

    public Command recoupleRotationCommand() {
        return runOnce(this::decoupleRotation);
    }

    public void setDesiredAngle(Rotation2d angle) {
        mDesiredAngle = angle;
    }

    private SwerveModulePosition[] getModulePositions() {
        return (SwerveModulePosition[]) Stream.of(mModules).map((m) -> m.getPosition()).toArray();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(mIMU.getAngle());
    }

    @Override
    public void periodic() {
        mPoseEstimator.update(getAngle(), getModulePositions());
    }
}
