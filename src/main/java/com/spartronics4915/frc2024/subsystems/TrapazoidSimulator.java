package com.spartronics4915.frc2024.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Trapzoid motion profiles act as the setpoint for the motors that the motors try to follow
 * this means that they act independently of the Motors meaning they can be simulator
 * this means that they can be simulated!
 */
public class TrapazoidSimulator extends SubsystemBase{

    public record SimulatorSettings(
        String name, double length, double angle, double lineWidth, Color8Bit color
    ) {}

    public interface TrapazoidSimulatorInterface {
        public State getSetPoint();
        public SimulatorSettings getSettings();
    }


    public record SimulatorObject(
        TrapazoidSimulatorInterface object, MechanismLigament2d Visual
    ) {}

    private ArrayList<SimulatorObject> SimulatedObjects;
    private Mechanism2d simCanvas;

    public TrapazoidSimulator(TrapazoidSimulatorInterface[] simulatedObjects) {

        simCanvas = new Mechanism2d(4, 4,  new Color8Bit("#1f0038"));
        for (TrapazoidSimulatorInterface simObj : simulatedObjects) {
            var settings = simObj.getSettings();
            var simLigament = new MechanismLigament2d(
                settings.name,
                settings.length,
                settings.angle,
                settings.lineWidth,
                settings.color
            );
            simCanvas.getRoot("Trapazoids", 3, 3).append(simLigament);
            SimulatedObjects.add(new SimulatorObject(simObj, simLigament));
        }
    } 
    
    @Override
    public void periodic() {
        
    }

}
