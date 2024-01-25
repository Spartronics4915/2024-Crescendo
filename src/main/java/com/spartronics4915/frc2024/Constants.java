package com.spartronics4915.frc2024;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.spartronics4915.frc2024.Constants.UtilRec.*;

import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
    public static final class OI {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kDriverTriggerDeadband = 0.3;
        public static final double kOperatorTriggerDeadband = 0.3;

        public static final int kIntakeBeamBreakID = 0; //placeholder
    }

    public static final class Drive {
        public static final int kPigeon2ID = 2;
    }

    public static final class GeneralConstants {
        public static final double kUpdateTime = 1.0/50.0; //used in trapazoid profiles
    }

    public static final class UtilRec {
        public static record PIDConstants(
                double P,
                double I,
                double D
            ) {}

        public static record MotorContstants(
            int kMotorID,
            MotorType kMotorType,
            boolean kMotorIsInverted, 
            IdleMode kIdleMode,
            int kCurrentLimit
        ) {}

        public static record TrapazoidConstaintsConstants(
            double kMaxVel,
            double kMaxAccel
        ) {}
    }
    public static final class IntakeAssembly {
        public enum IntakeAssemblyState{ //TODO find constants
            GROUNDPICKUP (Rotation2d.fromDegrees(0.0), 0.0),
            STOW (Rotation2d.fromDegrees(0.0), 0.0),
            AMP (Rotation2d.fromDegrees(0.0), 0.0),
            LOAD (Rotation2d.fromDegrees(0.0), 0.0),
            MANUAL (Rotation2d.fromDegrees(0.0), 0.0);

            public final Rotation2d wristAngle;
            public final double ElevatorHeight;
            private IntakeAssemblyState(Rotation2d wristAngle, double elevatorHeight) {
                this.wristAngle = wristAngle;
                ElevatorHeight = elevatorHeight;
            }
            
        }
        

        public static final class IntakeConstants {
            public static final MotorContstants kMotorConstants = new MotorContstants(15, MotorType.kBrushless, false, IdleMode.kBrake, 40);

            public static final PIDConstants kPIDconstants = new PIDConstants(1.0, 1.0, 1.0); //HACK DO NOT TEST WITH THESE VALUES
            

            public static final double kInSpeed = 0.2; // placeholder
            public static final double kLoadSpeed = 0.2; //placeholder
            public static final double kOutSpeed = -0.3; // placeholder
            public static final double kOffSpeed = 0;
        }
        
        public static final class IntakeWristConstants {

            //TODO Make Units Clear

            public static final MotorContstants kMotorConstants = new MotorContstants(15, MotorType.kBrushless, false, IdleMode.kBrake, 40);
            public static final PIDConstants kPIDconstants = new PIDConstants(1.0, 1.0, 1.0); //HACK DO NOT TEST WITH THESE VALUES
            public static final TrapazoidConstaintsConstants kTrapzoidConstants = new TrapazoidConstaintsConstants(10, 10); //HACK DO NOT TEST WITH THESE VALUES

            public static final IntakeAssemblyState kStartupState = IntakeAssemblyState.STOW;

            public static final class ManualConstants { //speed of manual movments
                
            }
        }

        public static final class ElevatorConstants {
            
        }
    }
}
