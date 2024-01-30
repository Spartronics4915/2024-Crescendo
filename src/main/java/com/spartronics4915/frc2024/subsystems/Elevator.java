package com.spartronics4915.frc2024.subsystems;

import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.SimulatorSettings;
import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.TrapazoidSimulatorInterface;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.util.TrapazoidSubsystemInterface;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements TrapazoidSimulatorInterface, TrapazoidSubsystemInterface {
    private static Elevator mInstance;

    public Elevator() {

    }

    @Override
    public void periodic() {
        
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

    public static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }
}
