package com.spartronics4915.frc2024.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;

import com.spartronics4915.frc2024.util.*;

import static com.spartronics4915.frc2024.Constants.Drive.*;

public class SwerveModule {
    private final CANSparkBase mDriveMotor;
    private final CANSparkBase mAngleMotor;

    private final SparkPIDController mDrivePID;
    private final SparkPIDController mAnglePID;

    private final SparkRelativeEncoder mDriveEncoder;

    private final CANcoder mCANCoder;

    private final double mX;
    private final double mY;

    private SwerveModuleState mLastDesiredStateSet;

    public SwerveModule(
            int driveMotorID,
            int angleMotorID,
            int encoderID,
            double encoderOffsetDegrees,
            double x,
            double y) {
        mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        configureDriveMotor(mDriveMotor);
        configureAngleMotor(mAngleMotor);

        mDrivePID = mDriveMotor.getPIDController();
        mAnglePID = mAngleMotor.getPIDController();

        mDriveEncoder = (SparkRelativeEncoder) mDriveMotor.getEncoder();

        mCANCoder = new CANcoder(encoderID);
        mCANCoder
                .getConfigurator()
                .apply(new CANcoderConfiguration()
                        .withMagnetSensor(new MagnetSensorConfigs()
                                .withMagnetOffset(encoderOffsetDegrees)));

        mX = x;
        mY = y;
    }

    public SwerveModule(ModuleConstants mc) {
        this(mc.driveMotorID(), mc.angleMotorID(), mc.encoderID(), mc.encoderOffsetDegrees(), mc.x(), mc.y());
    }

    public void setDesiredState(SwerveModuleState state) {
        mDrivePID.setReference(state.speedMetersPerSecond,
                ControlType.kSmartVelocity);
        mAnglePID.setReference(state.angle.getDegrees(),
                ControlType.kPosition);

        mLastDesiredStateSet = state;
    }

    public SwerveModuleState getDesiredState() {
        return mLastDesiredStateSet;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(mDriveEncoder.getVelocity(), getAngle());
    }

    /**
     * Only works in simulation.
     */
    @Deprecated(forRemoval = true)
    public void setPosition(double pos) {
        if (RobotBase.isSimulation()) {
            mDriveEncoder.setPosition(pos);
        }
    }

    /**
     * Only works in simulation.
     */
    public void setPosition(SwerveModulePosition pos) {
        if (RobotBase.isSimulation()) {
            mDriveEncoder.setPosition(pos.distanceMeters);
            mCANCoder.setPosition(pos.angle.getRotations());
        }
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(mDriveEncoder.getPosition(), getAngle());
    }

    public Translation2d getLocation() {
        return new Translation2d(mX, mY);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(mCANCoder.getPosition().getValue());
    }

    private void configureDriveMotor(CANSparkBase motor) {
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(kDriveMotorCurrentLimit);
        motor.enableVoltageCompensation(kMaxVoltage);
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kBrake);

        final var encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(kDrivePositionConversionFactor);
        encoder.setVelocityConversionFactor(kDriveVelocityConversionFactor);
        encoder.setPosition(0);

        final var pid = motor.getPIDController();
        final var pc = kDrivePIDFConstants;
        pid.setP(pc.p());
        pid.setI(pc.i());
        pid.setD(pc.d());
        pid.setFF(pc.ff());
        pid.setSmartMotionMaxVelocity(kMaxSpeed, 1);
        pid.setSmartMotionMaxAccel(kMaxAcceleration, 1);

        motor.burnFlash();
    }

    private void configureAngleMotor(CANSparkBase motor) {
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(kAngleMotorCurrentLimit);
        motor.enableVoltageCompensation(kMaxVoltage);
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kBrake);

        final var encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(kAnglePositionConversionFactor);
        encoder.setPosition(0);

        final var pid = motor.getPIDController();
        final var pc = kDrivePIDFConstants;
        pid.setP(pc.p());
        pid.setI(pc.i());
        pid.setD(pc.d());
        pid.setFF(pc.ff());
        pid.setPositionPIDWrappingEnabled(true);
        pid.setPositionPIDWrappingMaxInput(180);
        pid.setPositionPIDWrappingMinInput(-180);

        motor.burnFlash();
    }
}
