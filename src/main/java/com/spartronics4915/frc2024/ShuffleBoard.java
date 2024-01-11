package com.spartronics4915.frc2024;

import java.util.EnumMap;

import com.spartronics4915.frc2024.subsystems.Intake;
import com.spartronics4915.frc2024.subsystems.Intake.IntakeState;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class ShuffleBoard {
    public static String UserTab = "Overview"; //anything the drivers need to see should be on this tab
    public static String DebugTab = "Debug"; //anything that will need to be referenced for debugging should be on this tab
    
    public static class IntakeTabManager{
        public static String tabName = "Intake";

        public static enum IntakeSubsystemEntries{
            IntakeState ("State");

            private String entryName;
            private IntakeSubsystemEntries(String entryName) {this.entryName = entryName;}
        }

        public static EnumMap<IntakeSubsystemEntries, GenericEntry> getEnumMap(Intake subsystem) { 
                EnumMap<IntakeSubsystemEntries, GenericEntry> out = new EnumMap<>(IntakeSubsystemEntries.class);
                ShuffleboardLayout mIntakeOverview = Shuffleboard
                    .getTab("Overview")
                    .getLayout("Intake", BuiltInLayouts.kList)
                    .withSize(2, 2);

                
                out.put(IntakeSubsystemEntries.IntakeState, 
                    mIntakeOverview.add(IntakeSubsystemEntries.IntakeState.entryName, IntakeState.NONE.name())
                        .withSize(2, 2)
                        .getEntry()
                );

                return out;
            }
    }

}
