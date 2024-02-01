package com.spartronics4915.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.util.Loggable;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements Loggable {
    
    public static enum ShooterState {
        IN, LOAD, OUT, OFF, NONE; // NONE is only here as the Shuffleboard default value for troubleshooting
    }

    private static Shooter mInstance;

    private ShooterState mCurrentState;

    //private final CANSparkMax mShooterMotor;
 /* 
    public Shooter() {
        mCurrentState = ShooterState.OFF;
    }

    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    } */
    
    @Override
    public void updateShuffleboard() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateShuffleboard'");
    }

}
