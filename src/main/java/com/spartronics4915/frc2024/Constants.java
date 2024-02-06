package com.spartronics4915.frc2024;

import static com.spartronics4915.frc2024.Constants.Drive.kFrontLeft;

import java.util.stream.Stream;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.spartronics4915.frc2024.util.*;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
    public static final class OI {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kStickDeadband = 0.05;

        public static final double kDriverTriggerDeadband = 0.3;
        public static final double kOperatorTriggerDeadband = 0.3;
    }

    public static final class GeneralConstants {
        public static final double kUpdateTime = 1/50.0;
    }

    public static final class Drive {
        public static final int kPigeon2ID = 2;

        public static final PIDConstants kAngleControllerPIDConstants = new PIDConstants(5.0, 1.5, 2.0); // tuned good enough

        public static final Matrix<N3, N1> kStateStdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1, 0.1);
        public static final Matrix<N3, N1> kVisionMeasurementStdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1, 0.1);

        public static final double kWheelDiameter = Units.inchesToMeters(3.96);
        public static final double kTrackWidth = Units.inchesToMeters(18.75);
        public static final double kWheelbase = Units.inchesToMeters(23.75);
        public static final double kChassisRadius = Math.hypot(
                kTrackWidth / 2, kWheelbase / 2);

        public static final double kDriveGearRatio = 6.75 / 1.0; // L2 MK4i
        public static final double kDriveVelocityConversionFactor = ((kWheelDiameter * Math.PI) / kDriveGearRatio) / 60.0; // RPM to m/s
        public static final double kDrivePositionConversionFactor = ((kWheelDiameter * Math.PI) / kDriveGearRatio); // rev. to meters

        public static final double kAngleGearRatio =  150.0 / 7.0; // MK4i
        public static final double kAnglePositionConversionFactor = (2 * Math.PI) / (kAngleGearRatio);

        // Decrease this value if wheels start to slip with worn out tread. Should be 1.0 with new tread.
        public static final double kTreadWearAdjustment = 1.0;
        public static final double kTreadCoefficientOfFriction = 1.13; // black neoprene

        // theoretical maximum with NEO and L2 MK4i
        public static final double kMaxSpeed = Units.feetToMeters(15.1);
        public static final double kMaxAcceleration = 9.81 * kTreadCoefficientOfFriction * kTreadWearAdjustment;

        public static final double kMaxAngularSpeed = kMaxSpeed / kChassisRadius;
        public static final double kMaxAngularAcceleration = kMaxAcceleration / kChassisRadius;

        public static final PIDFConstants kDrivePIDFConstants = new PIDFConstants(0.0, 0.0, 0.0, 0.2); // placeholder values
        public static final PIDFConstants kAnglePIDFConstants = new PIDFConstants(1.0, 0.0, 0.5, 0.0); // placeholder values

        public static final ModuleConstants kFrontLeft = new ModuleConstants(
                5, 6, 13, 96.680, kWheelbase / 2, kTrackWidth / 2);

        public static final ModuleConstants kBackLeft = new ModuleConstants(
                7, 8, 14, 15.645, -kWheelbase / 2, kTrackWidth / 2);

        public static final ModuleConstants kBackRight = new ModuleConstants(
                9, 10, 11, 119.268, -kWheelbase / 2, -kTrackWidth / 2);

        public static final ModuleConstants kFrontRight = new ModuleConstants(
                3, 4, 12, 166.816, kWheelbase / 2, -kTrackWidth / 2);

        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
                (Translation2d[]) Stream.of(new ModuleConstants[] { kFrontLeft, kBackLeft, kBackRight, kFrontRight })
                .map((mc) -> new Translation2d(mc.x(), mc.y()))
                .toArray(Translation2d[]::new));

        public static final ReplanningConfig kReplanningConfig = new ReplanningConfig(true, true);
        public static final HolonomicPathFollowerConfig kPPConfig = new HolonomicPathFollowerConfig(
                new com.pathplanner.lib.util.PIDConstants(5.0, 0.0, 0.0),
                new com.pathplanner.lib.util.PIDConstants(5.0, 0.0, 0.0),
                kMaxSpeed,
                kChassisRadius,
                kReplanningConfig);

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

    public static final class IntakeAssembly {
        public enum IntakeAssemblyState{ //TODO find constants
            GROUNDPICKUP (Rotation2d.fromDegrees(0.0), 0.0),
            STOW (Rotation2d.fromDegrees(180.0), 0.0),
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
                    public static final MotorConstants kMotorConstants = new MotorConstants(0, MotorType.kBrushless, false, IdleMode.kBrake, 40);
        
                    public static final PIDConstants kPIDconstants = new PIDConstants(1.0, 1.0, 1.0); //HACK DO NOT TEST WITH THESE VALUES
                    
        
                    public static final double kInSpeed = 0.2; // placeholder
                    public static final double kLoadSpeed = 0.2; //placeholder
                    public static final double kOutSpeed = -0.3; // placeholder
                    public static final double kOffSpeed = 0;

                    public static final int kIntakeBeamBreakID = 0; //placeholder
                }
        
        public static final class IntakeWristConstants {

            //TODO Make Units Clear

            public static final MotorConstants kMotorConstants = new MotorConstants(1, MotorType.kBrushless, false, IdleMode.kBrake, 40);
            public static final PIDConstants kPIDconstants = new PIDConstants(1.0, 1.0, 1.0); //HACK DO NOT TEST WITH THESE VALUES
            public static final Constraints kTrapzoidConstraints = new Constraints(10, 10); //HACK DO NOT TEST WITH THESE VALUES

            // public static final IntakeAssemblyState kStartupState = IntakeAssemblyState.STOW;

            public static final class ManualConstants { //speed of manual movements, 
                
            }
        }

        public static final class ElevatorConstants {
            
        }
    }

    public static final class Vision {
        public enum VisionPipelines {
            FIDUCIALS_3D (0, false),
            DETECTOR_NOTE (1, true),
            DETECTOR_ROBOT (2, true),
            ALICE_TEMP_NOTE_DETECTOR (4, true); //HACK keep until alice is flashed
            public final int pipeline;
            public final boolean isDetector;
            private VisionPipelines(int pipeline, boolean isDetector) {
                this.pipeline = pipeline;
                this.isDetector = isDetector;
            }
        }

        public static final class PoseOffsetConstants {
            public static final Pose3d kAlicePoseOffset = new Pose3d(); // placeholder
            public static final Pose3d kBobPoseOffset = new Pose3d(); // placeholder
        }
    }
}
