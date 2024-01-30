package com.spartronics4915.frc2024.subsystems;

import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.SimType;
import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.SimulatorSettings;
import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.TrapazoidSimulatorInterface;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.spartronics4915.frc2024.util.TrapazoidSubsystemInterface;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2024.Constants.IntakeAssembly.ElevatorConstants.*;

// Name changed from Elevator to Elavetor in solidarity with Trapazoid
public class Elavetor extends SubsystemBase implements TrapazoidSimulatorInterface, TrapazoidSubsystemInterface {
    private static Elavetor mInstance;

    private CANSparkMax mMotor;

    public Elavetor() {
        mMotor = new CANSparkMax(kMotorId, MotorType.kBrushless);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void setPositionToReal() {

    }

    // todo: fix
    @Override
    public State getSetPoint() {
        return new State();
    }

    @Override
    public SimulatorSettings getSettings() {
        return new SimulatorSettings(
                "Elevator",
                1.0,
                90.0,
                20.0,
                new Color8Bit(Color.kBlanchedAlmond),
                SimType.Elevator,
                new Translation2d(103 / 100d, 27 / 100d));
    }

    public static Elavetor getInstance() {
        if (mInstance == null) {
            mInstance = new Elavetor();
        }
        return mInstance;
    }
}
