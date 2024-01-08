package com.spartronics4915.frc2024;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.spartronics4915.frc2024.Constants.UtilRec.*;

public final class Constants {
    public static final class OI {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kDriverTriggerDeadband = 0.3;
        public static final double kOperatorTriggerDeadband = 0.3;

        public static final int IntakeBeamBreakID = 0; //placeholder
    }

    public static final class Drive {
        public static final int kPigeon2ID = 2;
    }

    public static final class UtilRec {
        public static record PIDConstants(
                double kP,
                double kI,
                double kD
            ) {}

        public static record MotorContstants(
            int kMotorID,
            MotorType kMotorType,
            boolean kMotorIsInverted, 
            IdleMode kIdleMode,
            int kCurrentLimit
        ) {}
    }

    public static final class Intake {

        public static final MotorContstants kMotorConstants = new MotorContstants(15, MotorType.kBrushless, false, IdleMode.kBrake, 40);

        public static final int kMotorID = 15;
        public static final boolean kMotorIsInverted = false; // subject to change
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        public static final int kCurrentLimit = 40;

        public static final PIDConstants kPIDconstants = new PIDConstants(1.0, 1.0, 1.0); //FIXME DO NOT TEST WITH THESE VALUES
        

        public static final double kInSpeed = 0.2; // placeholder
        public static final double kOutSpeed = -0.3; // placeholder
        public static final double kOffSpeed = 0;
    }
}
