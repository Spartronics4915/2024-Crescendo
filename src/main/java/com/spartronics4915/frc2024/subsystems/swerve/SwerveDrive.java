package com.spartronics4915.frc2024.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.spartronics4915.frc2024.RobotContainer;
import com.spartronics4915.frc2024.subsystems.vision.VisionSubsystem;
import com.spartronics4915.frc2024.subsystems.vision.LimelightDevice.VisionMeasurement;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import static edu.wpi.first.math.MathUtil.applyDeadband;

import static com.spartronics4915.frc2024.Constants.Drive.*;
import static com.spartronics4915.frc2024.Constants.OI.kStickDeadband;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;

public class SwerveDrive extends SubsystemBase {
    private static SwerveDrive mInstance;

    private final SwerveModule mFrontLeft;
    private final SwerveModule mBackLeft;
    private final SwerveModule mBackRight;
    private final SwerveModule mFrontRight;

    private final SwerveModule[] mModules;

    private final Pigeon2 mIMU;

    private boolean mIsFieldRelative;

    private final PIDController mAngleController;
    private Rotation2d mDesiredAngle;
    private boolean mRotationIsIndependent;

    private final SwerveDrivePoseEstimator mPoseEstimator;

    private SwerveDrive() {
        mFrontLeft = new SwerveModule(kFrontLeft);
        mBackLeft = new SwerveModule(kBackLeft);
        mBackRight = new SwerveModule(kBackRight);
        mFrontRight = new SwerveModule(kFrontRight);

        mModules = new SwerveModule[] { mFrontLeft, mBackLeft, mBackRight, mFrontRight };

        mIMU = new Pigeon2(kPigeon2ID);

        mIsFieldRelative = true;

        {
            final var pc = kAngleControllerPIDConstants;
            mAngleController = new PIDController(pc.p(), pc.i(), pc.d());
        }

        mRotationIsIndependent = false;

        {
            final var stateStdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1, 0.1);
            final var visionMeasurementStdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1, 0.1);

            // TODO: change initial pose estimate
            mPoseEstimator = new SwerveDrivePoseEstimator(kKinematics, getAngle(), getModulePositions(), new Pose2d(),
                    stateStdDevs, visionMeasurementStdDevs);
        }

        setDefaultCommand(teleopDriveCommand());
    }

    /**
     * Returns the current instance of {@link SwerveDrive}.
     */
    public static SwerveDrive getInstance() {
        if (mInstance == null) {
            mInstance = new SwerveDrive();
        }
        return mInstance;
    }

    /**
     * Drives the robot given a {@link ChassisSpeeds} and whether to drive field relative or not.
     */
    public void drive(final ChassisSpeeds speeds, final boolean fieldRelative) {
        drive(speeds, fieldRelative, mRotationIsIndependent);
    }

    // FIXME: field relative drives faster than robot relative (??)
    // FIXME: need to make field relative based on pose and mirror based on alliance
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

        final var moduleStates = kKinematics.toSwerveModuleStates(_speeds);

        final var moduleStatesIterator = List.of(moduleStates).iterator();

        for (SwerveModule m : mModules) {
            m.setDesiredState(SwerveModuleState.optimize(moduleStatesIterator.next(), m.getState().angle));
        }
    }

    private Command teleopDriveCommand() {
        final var swerve = this;
        return new Command() {
            {
                addRequirements(swerve);
            }

            @Override
            public void execute() {
                final var dc = RobotContainer.getDriverController();
                ChassisSpeeds cs = new ChassisSpeeds();

                final double inputxraw = dc.getLeftY() * -1.0;
                final double inputyraw = dc.getLeftX() * -1.0;
                final double inputomegaraw = dc.getRightX() * -1.0; // consider changing from angular velocity control to direct angle control

                final double inputx = applyResponseCurve(applyDeadband(inputxraw, kStickDeadband));
                final double inputy = applyResponseCurve(applyDeadband(inputyraw, kStickDeadband));
                final double inputomega = applyResponseCurve(applyDeadband(inputomegaraw, kStickDeadband));

                cs.vxMetersPerSecond = inputx * kMaxSpeed;
                cs.vyMetersPerSecond = inputy * kMaxSpeed;
                cs.omegaRadiansPerSecond = inputomega * kMaxAngularSpeed;
                
                // cs.vxMetersPerSecond = 1;
                // cs.vyMetersPerSecond = 1;
                // cs.omegaRadiansPerSecond = 0;

                drive(cs, mIsFieldRelative);
            }

            private double applyResponseCurve(double x) {
                return Math.signum(x) * Math.pow(x, 2);
            }
        };
    }

    /**
     * Toggles whether the robot will drive field relative by default.
     */
    public void toggleFieldRelative() {
        mIsFieldRelative = !mIsFieldRelative;
    }

    /**
     * Toggles whether the robot will drive field relative by default.
     */
    public Command toggleFieldRelativeCommand() {
        return runOnce(this::toggleFieldRelative);
    }

    /**
     * Sets whether the robot will drive field relative by default.
     */
    public void setFieldRelative(boolean fieldRelative) {
        mIsFieldRelative = fieldRelative;
    }

    /**
     * Returns whether the robot will drive field relative by default.
     */
    public boolean isFieldRelative() {
        return mIsFieldRelative;
    }

    /**
     * Returns whether the rotation is decoupled.
     */
    public boolean rotationIsDecoupled() {
        return mRotationIsIndependent;
    }

    /**
     * Decouples the rotation control from any drive commands.
     */
    public void decoupleRotation() {
        setDesiredAngle(getAngle());
        mRotationIsIndependent = true;
    }

    /**
     * Decouples the rotation control from any drive commands.
     */
    public Command decoupleRotationCommand() {
        return runOnce(this::decoupleRotation);
    }

    /**
     * Recouples the rotation control to any drive commands.
     */
    public void recoupleRotation() {
        mRotationIsIndependent = false;
    }

    /**
     * Recouples the rotation control to any drive commands.
     */
    public Command recoupleRotationCommand() {
        return runOnce(this::recoupleRotation);
    }

    /**
     * Sets the desired heading angle of the swerve drive. Only works if rotation is decoupled.
     */
    public void setDesiredAngle(final Rotation2d angle) {
        mDesiredAngle = angle;
    }

    private SwerveModulePosition[] getModulePositions() {
        return (SwerveModulePosition[]) Stream.of(mModules).map((m) -> m.getPosition()).toArray(SwerveModulePosition[]::new);
    }

    /**
     * Returns the desired {@link SwerveModuleState}s of the swerve modules.
     */
    public SwerveModuleState[] getModuleDesiredStates() {
        return (SwerveModuleState[]) Stream.of(mModules).map((m) -> m.getDesiredState()).toArray(SwerveModuleState[]::new);
    }

    /**
     * Gets the angle from the IMU.
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(mIMU.getAngle() * -1.0);
    }

    /**
     * Adds a vision measurement to the pose estimator.
     */
    public void addVisionMeasurement(final VisionMeasurement msmt) {
        final var p = msmt.pose().toPose2d();
        final var t = msmt.timestamp();
        addVisionMeasurement(p, t);
    }

    /**
     * Adds a vision measurement to the pose estimator.
     */
    public void addVisionMeasurement(final Pose2d cameraPose, final double t) {
        mPoseEstimator.addVisionMeasurement(cameraPose, t);
    }

    /**
     * Gets the pose estimator's estimated pose.
     */
    public Pose2d getPose() {
        return mPoseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the pose estimator to the specified position.
     */
    public void resetPose(final Pose2d newPose) {
        mPoseEstimator.resetPosition(getAngle(), getModulePositions(), newPose);
    }

    @Override
    public void periodic() {
        mPoseEstimator.update(getAngle(), getModulePositions());
        boolean logSwerveModules = true;
        if (logSwerveModules) {
            for(int i = 0; i < 4; i ++){
                String moduleTag = "Module " + i + " encoder : ";
                double encoderReading = mModules[i].getPosition().angle.getDegrees();
                SmartDashboard.putNumber(moduleTag, encoderReading);

                String moduleTagRaw = "Module " + i + " encoder raw: ";
                double encoderReadingRaw = mModules[i].getAbsoluteAngle().getDegrees();
                SmartDashboard.putNumber(moduleTagRaw, encoderReadingRaw);
            }
        }

        SmartDashboard.putNumber("IMU Yaw Degrees", getAngle().getDegrees());
        SmartDashboard.putNumber("Pose x", getPose().getX());
        SmartDashboard.putNumber("Pose y", getPose().getY());

        // This code causes the robot to crash
        
        // final var vs = VisionSubsystem.getInstance();
        // vs.getAlice().getVisionMeasurement().ifPresent(this::addVisionMeasurement);
        // vs.getBob().getVisionMeasurement().ifPresent(this::addVisionMeasurement);
    }

    public SwerveModule[] getSwerveModules() {

        return mModules;
    }
    
    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return kKinematics;
    }

    public Pigeon2 getIMU() {
        return mIMU;
    }
}
