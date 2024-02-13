package com.spartronics4915.frc2024;

import static com.spartronics4915.frc2024.Constants.Drive.kFrontLeft;

import java.util.stream.Stream;

import org.ddogleg.solver.RootFinderType;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.SimType;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.SimulatorSettings;
import com.spartronics4915.frc2024.util.*;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
        public static final double kUpdateTime = 1 / 50.0;
    }

    public static final class Drive {
        public static final int kPigeon2ID = 2;

        public static final PIDConstants kAngleControllerPIDConstants = new PIDConstants(5.0, 1.5, 2.0); // tuned good enough

        public static final Matrix<N3, N1> kStateStdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1, 0.1);
        public static final Matrix<N3, N1> kVisionMeasurementStdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1,
                0.1);

        public static final double kWheelDiameter = Units.inchesToMeters(3.87);
        public static final double kTrackWidth = Units.inchesToMeters(22.475);
        public static final double kWheelbase = Units.inchesToMeters(22.475);
        public static final double kChassisRadius = Math.hypot(
                kTrackWidth / 2, kWheelbase / 2);

        public static final double kDriveGearRatio = 6.75 / 1.0; // L2 MK4i
        public static final double kDriveVelocityConversionFactor = ((kWheelDiameter * Math.PI) / kDriveGearRatio)
                / 60.0; // RPM to m/s
        public static final double kDrivePositionConversionFactor = ((kWheelDiameter * Math.PI) / kDriveGearRatio); // rev.
                                                                                                                    // to
                                                                                                                    // meters

        public static final double kAngleGearRatio = 150.0 / 7.0; // MK4i
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

        public static final double kOdometryUpdatePeriod = 0.01;

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
                double y) {}
    }

    public static final class IntakeAssembly { // [ ] Intake Assembly constants
        public enum IntakeAssemblyState{ //TODO find constants
            GROUNDPICKUP (Rotation2d.fromDegrees(90.0), 1.0),
            STOW (Rotation2d.fromDegrees(170.0), 0.0),
            AMP (Rotation2d.fromDegrees(90.0), 0.5),
            LOAD (Rotation2d.fromDegrees(190.0), 0.0),
            SOURCE (Rotation2d.fromDegrees(120.0), 0.75),
            MANUAL (Rotation2d.fromDegrees(0.0), -0.1); //CHECKUP is this needed?

            public final Rotation2d wristAngle;
            public final double ElevatorHeight;

            private IntakeAssemblyState(Rotation2d wristAngle, double elevatorHeight) {
                this.wristAngle = wristAngle;
                ElevatorHeight = elevatorHeight;
            }
            
        }
        
        public static final class IntakeConstants {// [ ] Intake constants
                    public static final MotorConstants kMotorConstants = new MotorConstants(12, MotorType.kBrushless, false, IdleMode.kBrake, 40);
        
                    public static final PIDConstants kPIDconstants = new PIDConstants(1.0, 1.0, 1.0); //HACK DO NOT TEST WITH THESE VALUES
                    
        
                    public static final double kInSpeed = 0.2; // placeholder
                    public static final double kLoadSpeed = 0.2; //placeholder
                    public static final double kOutSpeed = -0.3; // placeholder
                    public static final double kOffSpeed = 0;

                    public static final int kIntakeBeamBreakID = 3; //placeholder
                }
        
        public static final class IntakeWristConstants { // [ ] Intake Wrist constants

            // TODO Make Units Clear

            public static final MotorConstants kMotorConstants = new MotorConstants(13, MotorType.kBrushless, false, IdleMode.kBrake, 40);
            public static final PIDConstants kPIDConstants = new PIDConstants(0.25, 0.0, 0.0); //HACK DO NOT TEST WITH THESE VALUES
            public static final Constraints kTrapzoidConstraints = new Constraints(1, 1); //HACK DO NOT TEST WITH THESE VALUES

            public static final double kInToOutRotations = 1.0;
            
            // public static final IntakeAssemblyState kStartupState = IntakeAssemblyState.STOW;

            public static final double kMeterSafetyLimit = 1.0; //HACK untested

            public static final Rotation2d kMaxAngleAmp = Rotation2d.fromDegrees(90); //only when above the safety height
            public static final Rotation2d kMaxAngleGround = Rotation2d.fromDegrees(170); //only when below the safety height
            public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(70); //FIXME used for sim

            public static final FeedForwardConstants kArmFeedForward = new FeedForwardConstants(1.0, 1.0, 1.0, 0.0); // HACK untested values

            public static final int kLimitSwitchChannel = 4;
            public static final double kLimitSwitchEncoderReading = 0;
            public static final class ManualConstants { // speed of manual movements,

            }
        }

        public static final class ElevatorConstants {
            public static final MotorConstants kMotorConstants = new MotorConstants(20, MotorType.kBrushless, false,
                    IdleMode.kBrake, 40);
            public static final MotorConstants kFollowerConstants = new MotorConstants(21, MotorType.kBrushless, false,
                    IdleMode.kBrake, 40); // HACK untested
            public static final Constraints kZoidConstants = new Constraints(1d, 1d);
            public static final double kMetersToRotation = 1; // Conversion rate
            public static final SimulatorSettings kElevatorSimulatorSettings = new SimulatorSettings(
                    "Elevator",
                    1.0,
                    90.0,
                    5.0,
                    new Color8Bit(Color.kMediumPurple),
                    SimType.Elevator,
                    new Translation2d(2, 0));
            public static final FeedForwardConstants kElevatorFeedFowardConstants = new FeedForwardConstants(.1026,
                    .0156, 7, 102); // HACK untested
            public static final int kLimitSwitchChannel = 2;
            public static final double kLimitSwitchGoto = 0; // Where the elevator will go to if the limit switch is
                                                             // triggered
            public static final double kMinimumManualRotations = 0.1;
            public static final double kMaximumRotations = 1.5; // hack untested
        }
    }

    public static final class ShooterWristConstants { // [ ] Shooter Wrist Constants
        public enum ShooterWristState{ //Mostly for debug
            SubwooferShot(Rotation2d.fromDegrees(90));

            public final Rotation2d shooterAngle;
            private ShooterWristState(Rotation2d shooterAngle) {
                this.shooterAngle = shooterAngle;
            }
        }

        public static double kInToOutRotations = 1.0;

        public static final MotorConstants kMotorConstants = new MotorConstants(11, MotorType.kBrushless, false, IdleMode.kBrake, 40); //placeholder
        public static final PIDConstants kPIDconstants = new PIDConstants(1.0, 1.0, 1.0); //don't test with these values
        public static final Constraints kTrapzoidConstants = new Constraints(1, 1); //HACK DO NOT TEST WITH THESE VALUES

        
        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(90); //only when above the safety height
        public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(5); //FIXME used for sim

        public static final FeedForwardConstants kWristFeedForward = new FeedForwardConstants(1.0, 1.0, 1.0, 0.0); //HACK untested values
        
        public static final double kAimedAtTargetThreshold = 1/60; //6 degrees

        public static final int kLimitSwitchChannel = 1;
        public static final double kLimitSwitchEncoderReading = 0;
    }

    public static final class ShooterConstants { // [ ] Shooter Constants
        public static final MotorConstants kShooterMotorConstants = new MotorConstants(24, MotorType.kBrushless, false, IdleMode.kCoast, 30); //placeholder
        public static final MotorConstants kShooterFollowMotorConstants = new MotorConstants(25, MotorType.kBrushless, false, IdleMode.kCoast, 30); //placeholder
        public static final MotorConstants kConveyorMotorConstants = new MotorConstants(26, MotorType.kBrushless, false, IdleMode.kCoast, 30); //placeholder
        public static final PIDFConstants kPIDconstants = new PIDFConstants(0.1, 0.0, 0.0, 0.0); //TODO find these values
        public static final double kOffSpeed = 0.0; //unsure if this is necessary
        public static final double kShootSpeed = 1.0; //placeholder
        public static final double kDiff = 0.05;
        public static final double kConveyorInSpeed = -1.0; //placeholder
        public static final double kConveyorOutSpeed = 1.0; //placeholder

        public static final double kTargetRPM = 1000;
    }

    public static final class AutoAimConstants {
        public enum Targets{
            TEMP_TARGET (new Translation3d(0.2286, 5.5, 2.0)),
            BLUE_SPEAKER (new Translation3d(0.2286, 5.5, 2.0)),
            RED_SPEAKER (new Translation3d(0.2286, 5.5, 2.0)); //GET RED SPEAKER POSITION


            public final Translation3d targetPosition;
            private Targets(Translation3d targetPosition){
                this.targetPosition = targetPosition;
            }
        }
        public static final Translation3d kStageTarget;
        static{
            if (DriverStation.getAlliance().isPresent()){
                switch (DriverStation.getAlliance().get()) {
                    case Blue:
                        kStageTarget = Targets.BLUE_SPEAKER.targetPosition;
                        break;
                    case Red:
                        kStageTarget = Targets.RED_SPEAKER.targetPosition;
                        break;
                    default:
                        kStageTarget = Targets.BLUE_SPEAKER.targetPosition;
                        break;
                }
            }else{

                kStageTarget = Targets.TEMP_TARGET.targetPosition; //TODO add alliance checker
            }
        }

        public static final Translation3d kAutoAimTarget = new Translation3d(0.2286, 5.5, 2.0);
        // public static final Translation3d kAutoAimTarget = new Translation3d(5, 5, 0.1);

        public static final double kShooterSpeed = 18.0; //needs to be in m/s
        public static final double kShooterHeight = 0.0;
        public static final double kMaxDistance = 10.0; //Needs units, the maximum relative distance a target can be from the robot for autoaim 
        public static final double kGravity = -9.8;
        public static final RootFinderType kRootFinderType = RootFinderType.STURM;
    }
    public static final class Vision {
        public enum VisionPipelines {
            FIDUCIALS_3D (0, false),
            DETECTOR_NOTE (1, true);
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
