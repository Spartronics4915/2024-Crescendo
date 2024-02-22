package com.spartronics4915.frc2024;

import java.util.EnumMap;

import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.subsystems.Elevator;
import com.spartronics4915.frc2024.subsystems.Shooter;
import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake.IntakeState;
import com.spartronics4915.frc2024.subsystems.Shooter.ShooterState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleBoard {
    public static String UserTab = "Overview"; //anything the drivers need to see should be on this tab
    public static String DebugTab = "Debug"; //anything that will need to be referenced for debugging should be on this tab

    public static <T extends Enum<T>> void putEntry(EnumMap<T, GenericEntry> map, T enumValue, Object defualtValue, ShuffleboardContainer shuffleContainer, String name){
        map.put(enumValue, shuffleContainer.add(name, defualtValue).withSize(2, 2).withPosition(0, 0).getEntry());
    }
    
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
                .getTab(tabName)
                .getLayout("Intake", BuiltInLayouts.kList)
                .withSize(2, 2);

            putEntry(out, IntakeSubsystemEntries.IntakeState, IntakeState.NONE, mIntakeOverview, IntakeSubsystemEntries.IntakeState.entryName);

            return out;
        }
    }

    public static class IntakeWristTabManager{
        public static String tabName = "IntakeWrist";

        public static enum WristSubsystemEntries{
            WristSetPoint ("WristSetPoint"),
            WristEncoderReading("WristEncoderReading"),
            WristManualControl ("WristManual");

            private String entryName;
            private WristSubsystemEntries(String entryName) {this.entryName = entryName;}
        }

        public static EnumMap<WristSubsystemEntries, GenericEntry> getEnumMap(IntakeWrist subsystem) { 
            EnumMap<WristSubsystemEntries, GenericEntry> out = new EnumMap<>(WristSubsystemEntries.class);
            ShuffleboardLayout mShuffleBoardTab = Shuffleboard
                .getTab(tabName)
                .getLayout(tabName+"layout`", BuiltInLayouts.kList)
                .withSize(2, 2);
        
            putEntry(out, WristSubsystemEntries.WristSetPoint, -1.0, mShuffleBoardTab, WristSubsystemEntries.WristSetPoint.entryName);

            putEntry(out, WristSubsystemEntries.WristManualControl, false, mShuffleBoardTab, WristSubsystemEntries.WristManualControl.entryName);

            putEntry(out, WristSubsystemEntries.WristEncoderReading, -1.0, mShuffleBoardTab, WristSubsystemEntries.WristEncoderReading.entryName);
    
            mShuffleBoardTab.add("stow", subsystem.setStateCommand(IntakeAssemblyState.STOW));
            mShuffleBoardTab.add("Amp", subsystem.setStateCommand(IntakeAssemblyState.AMP));

            return out;
        }
    }
    

    public static class ShooterTabManager{
        public static String tabName = "Intake";

        public static enum ShooterSubsystemEntries{
            ShooterState ("State");

            private String entryName;
            private ShooterSubsystemEntries(String entryName) {this.entryName = entryName;}
        }

        public static EnumMap<ShooterSubsystemEntries, GenericEntry> getEnumMap(Shooter subsystem) { 
            EnumMap<ShooterSubsystemEntries, GenericEntry> out = new EnumMap<>(ShooterSubsystemEntries.class);
            ShuffleboardLayout mIntakeOverview = Shuffleboard
                .getTab(tabName)
                .getLayout("Intake", BuiltInLayouts.kList)
                .withSize(2, 2);

            putEntry(out, ShooterSubsystemEntries.ShooterState, ShooterState.NONE, mIntakeOverview, ShooterSubsystemEntries.ShooterState.entryName);

            return out;
        }
    }

    public static class ShooterWristTabManager{
        public static String tabName = "ShooterWrist";

        public static enum ShooterWristSubsystemEntries{
            ShooterSetPoint ("ShooterSetPoint"),
            ShooterEncoderReading("ShooterEncoderReading"),
            ShooterManualControl ("ShooterManual");

            private String entryName;
            private ShooterWristSubsystemEntries(String entryName) {this.entryName = entryName;}
        }

        public static EnumMap<ShooterWristSubsystemEntries, GenericEntry> getEnumMap(ShooterWrist subsystem) { 
            EnumMap<ShooterWristSubsystemEntries, GenericEntry> out = new EnumMap<>(ShooterWristSubsystemEntries.class);
            ShuffleboardLayout mShuffleBoardTab = Shuffleboard
                .getTab(tabName)
                .getLayout(tabName+"layout`", BuiltInLayouts.kList)
                .withSize(2, 2);

            putEntry(out, ShooterWristSubsystemEntries.ShooterSetPoint, -1.0, mShuffleBoardTab, ShooterWristSubsystemEntries.ShooterSetPoint.entryName);

            putEntry(out, ShooterWristSubsystemEntries.ShooterManualControl, false, mShuffleBoardTab, ShooterWristSubsystemEntries.ShooterManualControl.entryName);

            putEntry(out, ShooterWristSubsystemEntries.ShooterEncoderReading, -1.0, mShuffleBoardTab, ShooterWristSubsystemEntries.ShooterEncoderReading.entryName);

            return out;
        }
    }

    
    public static class ElevatorTabManager{
        public static String tabName = "Elevator";

        public static enum ElevatorSubsystemEntries{
            ElevatorSetPoint ("ElevatorSetPoint"),
            ElevatorHeight("ElevatorHeight"),
            ElevatorManualControl ("ElevatorManual"),
            EncoderRawValue ("RawMain"),
            FollowerRawValue ("RawFollower");

            private String entryName;
            private ElevatorSubsystemEntries(String entryName) {this.entryName = entryName;}
        }

        public static EnumMap<ElevatorSubsystemEntries, GenericEntry> getEnumMap(Elevator subsystem) { 
            EnumMap<ElevatorSubsystemEntries, GenericEntry> out = new EnumMap<>(ElevatorSubsystemEntries.class);
            ShuffleboardLayout mShuffleBoardTab = Shuffleboard
                .getTab(tabName)
                .getLayout(tabName+"layout`", BuiltInLayouts.kList)
                .withSize(2, 2);
        
            putEntry(out, ElevatorSubsystemEntries.ElevatorSetPoint, -1.0, mShuffleBoardTab, ElevatorSubsystemEntries.ElevatorSetPoint.entryName);

            putEntry(out, ElevatorSubsystemEntries.ElevatorManualControl, false, mShuffleBoardTab, ElevatorSubsystemEntries.ElevatorManualControl.entryName);

            putEntry(out, ElevatorSubsystemEntries.ElevatorHeight, -1.0, mShuffleBoardTab, ElevatorSubsystemEntries.ElevatorHeight.entryName);

            return out;
        }
    }
    

}
