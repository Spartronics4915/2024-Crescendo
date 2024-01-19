package com.spartronics4915.frc2024;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.spartronics4915.frc2024.util.*;

import edu.wpi.first.math.util.Units;

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

        public static final double kWheelDiameter = Units.inchesToMeters(4);
        public static final double kTrackWidth = Units.inchesToMeters(22.475);
        public static final double kWheelbase = Units.inchesToMeters(22.475);
        public static final double kChassisRadius = Math.hypot(
                kTrackWidth / 2, kWheelbase / 2);

        // black neoprene
        public static final double kTreadCoefficientOfFriction = 1.13;
        // Incl. battery and bumpers, could be used to calculate required power for some acceleration. TODO: get actual value
        public static final double kRobotTotalMassKg = 50;

        public static final double kDriveGearRatio = 6.75 / 1.0; // L2 MK4i
        public static final double kDriveVelocityConversionFactor = ((kWheelDiameter * Math.PI) / kDriveGearRatio) / 60.0;
        public static final double kDrivePositionConversionFactor = ((kWheelDiameter * Math.PI) / kDriveGearRatio);

        public static final double kAngleGearRatio =  150.0 / 7.0; // MK4i
        public static final double kAnglePositionConversionFactor = (2 * Math.PI) / (kAngleGearRatio);
        // Decrease this value if wheels start to slip with worn out tread. Should be 1.0 with new tread.
        public static final double kTreadWearAdjustment = 1.0;

        // theoretical maximum with NEO Vortex and L2 MK4i
        public static final double kMaxSpeed = Units.feetToMeters(17.6);
        public static final double kMaxAcceleration = 9.81 * kTreadCoefficientOfFriction * kTreadWearAdjustment;

        public static final PIDFConstants kDrivePIDFConstants = new PIDFConstants(1.0, 0.0, 0.0, 0.0); // placeholder values
        public static final PIDFConstants kAnglePIDFConstants = new PIDFConstants(1.0, 0.0, 0.0, 0.0); // placeholder values

        public static final ModuleConstants kFrontLeft = new ModuleConstants(
                3, 4, 11, 0.0, kWheelbase / 2, kTrackWidth / 2);

        public static final ModuleConstants kBackLeft = new ModuleConstants(
                5, 6, 12, 0.0, -kWheelbase / 2, kTrackWidth / 2);

        public static final ModuleConstants kBackRight = new ModuleConstants(
                7, 8, 13, 0.0, -kWheelbase / 2, -kTrackWidth / 2);

        public static final ModuleConstants kFrontRight = new ModuleConstants(
                9, 10, 14, 0.0, kWheelbase / 2, -kTrackWidth / 2);

        public static final int kDriveMotorCurrentLimit = 40;
        public static final int kAngleMotorCurrentLimit = 40;
        public static final double kMaxVoltage = 12.0;

        public static final record ModuleConstants(
                int driveMotorID,
                int angleMotorID,
                int encoderID,
                double encoderOffsetDegrees,
                double x,
                double y
        ) {}
    }

    public static final class Intake {

        public static final MotorConstants kMotorConstants = new MotorConstants(15, MotorType.kBrushless, false, IdleMode.kBrake, 40);

        public static final int kMotorID = 15;
        public static final boolean kMotorIsInverted = false; // subject to change
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        public static final int kCurrentLimit = 40;

        public static final PIDConstants kPIDconstants = new PIDConstants(1.0, 1.0, 1.0); //FIXME DO NOT TEST WITH THESE VALUES

        public static final double kInSpeed = 0.2; // placeholder
        public static final double kLoadSpeed = 0.2; //placeholder
        public static final double kOutSpeed = -0.3; // placeholder
        public static final double kOffSpeed = 0;
    }
}
