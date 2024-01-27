package com.spartronics4915.frc2024.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

/**
 * Trapzoid motion profiles act as the setpoint for the motors that the motors try to follow
 * this means that they act independently of the Motors meaning they can be simulator
 * this means that they can be simulated!
 */
public class TrapazoidSimulator extends SubsystemBase{

    public enum SimType{
        Elevator,
        Angle,
    }
    
    public record SimulatorSettings(
        String name, double length, double angle, double lineWidth, Color8Bit color, SimType type, Translation2d rootPos
    ) {}

    public interface TrapazoidSimulatorInterface {
        public State getSetPoint();
        public SimulatorSettings getSettings();
    }


    public record SimulatorObject(
        TrapazoidSimulatorInterface object, 
        MechanismLigament2d visual,
        SimType type
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
        
            var simBase = simCanvas.getRoot(
                settings.name, 
                settings.rootPos.getX(), 
                settings.rootPos.getY()
            );

            simBase.append(simLigament);
            SimulatedObjects.add(new SimulatorObject(simObj, simLigament, settings.type));
        }
    } 
    
    @Override
    public void periodic() {
        for (var simobj : SimulatedObjects) {
            switch (simobj.type) {
                case Angle:
                    simobj.visual.setLength(simobj.object.getSetPoint().position);
                    break;
                case Elevator:
                    simobj.visual.setLength(simobj.object.getSetPoint().position);
                    break;
                default:
                    break;
            }
        }
    }

}
