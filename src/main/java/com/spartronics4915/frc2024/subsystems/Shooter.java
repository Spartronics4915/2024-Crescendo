package com.spartronics4915.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.SimulatorSettings;
import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.TrapazoidSimulatorInterface;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;
import com.spartronics4915.frc2024.util.TrapazoidSubsystemInterface;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements TrapazoidSimulatorInterface, TrapazoidSubsystemInterface {
    
    private CANSparkMax mShooterMotor;

     private static Shooter mInstance;

     public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    @Override
    public void setPositionToReal() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPositionToReal'");
    }

    @Override
    public State getSetPoint() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSetPoint'");
    }

    @Override
    public SimulatorSettings getSettings() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSettings'");
    }

}
