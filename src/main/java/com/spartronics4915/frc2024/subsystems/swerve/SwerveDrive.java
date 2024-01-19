package com.spartronics4915.frc2024.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2024.Constants.Drive.*;

public class SwerveDrive extends SubsystemBase {
    private SwerveDrive mInstance;

    private final SwerveModule mFrontLeft;
    private final SwerveModule mBackLeft;
    private final SwerveModule mBackRight;
    private final SwerveModule mFrontRight;

    private final Pigeon2 mIMU;

    private SwerveDrive() {
        mFrontLeft = new SwerveModule(kFrontLeft);
        mBackLeft = new SwerveModule(kBackLeft);
        mBackRight = new SwerveModule(kBackRight);
        mFrontRight = new SwerveModule(kFrontRight);

        mIMU = new Pigeon2(kPigeon2ID);
    }

    public SwerveDrive getInstance() {
        if (mInstance == null) {
            mInstance = new SwerveDrive();
        }
        return mInstance;
    }
}
