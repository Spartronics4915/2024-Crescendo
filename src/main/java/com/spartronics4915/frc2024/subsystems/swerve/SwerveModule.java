package com.spartronics4915.frc2024.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final CANSparkMax mDriveMotor;
    private final CANSparkMax mAngleMotor;

    private final double mX;
    private final double mY;

    private SwerveModuleState mDesiredState;

    public SwerveModule(
            int driveMotorID,
            int angleMotorID,
            double x,
            double y) {
        mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        // TODO: set settings

        mX = x;
        mY = y;

        mDesiredState = new SwerveModuleState();
    }

    public void setDesiredState(SwerveModuleState state) {
        mDesiredState = state;
    }
}
