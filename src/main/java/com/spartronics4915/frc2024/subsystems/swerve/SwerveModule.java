package com.spartronics4915.frc2024.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final CANSparkFlex mDriveMotor;
    private final CANSparkMax mAngleMotor;

    private final CANcoder mCANCoder;

    private final double mX;
    private final double mY;

    private SwerveModuleState mDesiredState;

    public SwerveModule(
            int driveMotorID,
            int angleMotorID,
            int encoderID,
            double x,
            double y) {
        mDriveMotor = new CANSparkFlex(driveMotorID, MotorType.kBrushless);
        mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        // TODO: set settings

        mCANCoder = new CANcoder(encoderID);

        mX = x;
        mY = y;

        mDesiredState = new SwerveModuleState();
    }

    public void setDesiredState(SwerveModuleState state) {
        mDesiredState = state;
    }
}
