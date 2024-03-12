package com.spartronics4915.frc2024;

import static com.spartronics4915.frc2024.Constants.Drive.kFrontLeft;
import static com.spartronics4915.frc2024.Constants.IntakeAssembly.ElevatorConstants.kMetersToRotation;
import static com.spartronics4915.frc2024.Constants.ShooterConstants.kShootSpeed;
import static com.spartronics4915.frc2024.Constants.ShooterConstants.kTargetRPM;

import java.util.stream.Stream;

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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

        public static final PIDConstants kAngleControllerPIDConstants = new PIDConstants(9.0, 0.0, 0.0); // new
                                                                                                         // PIDConstants(5.0,
                                                                                                         // 1.5, 2.0);
                                                                                                         // // tuned
                                                                                                         // good enough

        public static final Matrix<N3, N1> kStateStdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1, 0.1);
        public static final Matrix<N3, N1> kVisionMeasurementStdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1,
                0.1);

        public static final double kWheelDiameter = Units.inchesToMeters(3.96);
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
        public static final double kMaxAcceleration = 5; // 9.81 * kTreadCoefficientOfFriction * kTreadWearAdjustment;

        public static final double kMaxAngularSpeed = kMaxSpeed / kChassisRadius;
        public static final double kMaxAngularAcceleration = kMaxAcceleration / kChassisRadius;

        public static final PIDFConstants kDrivePIDFConstants = new PIDFConstants(0.0, 0.0, 0.0, 0.2); // placeholder
                                                                                                       // values
        public static final PIDFConstants kAnglePIDFConstants = new PIDFConstants(1.0, 0.0, 0.5, 0.0); // placeholder
                                                                                                       // values

        public static final ModuleConstants kFrontLeft = new ModuleConstants(
                3, 4, 11, -91.318, kWheelbase / 2, kTrackWidth / 2);

        public static final ModuleConstants kBackLeft = new ModuleConstants(
                5, 6, 12, 74.268, -kWheelbase / 2, kTrackWidth / 2);

        public static final ModuleConstants kBackRight = new ModuleConstants(
                7, 8, 13, 286.699, -kWheelbase / 2, -kTrackWidth / 2);

        public static final ModuleConstants kFrontRight = new ModuleConstants(
                9, 10, 14, 166.377, kWheelbase / 2, -kTrackWidth / 2);

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
        public static final int kAngleMotorCurrentLimit = 25;
        public static final double kMaxVoltage = 12.0;

        public static final Rotation2d kAimedAtTargetThreshold = Rotation2d.fromDegrees(0.05);

        public static final record ModuleConstants(
                int driveMotorID,
                int angleMotorID,
                int encoderID,
                double encoderOffsetDegrees,
                double x,
                double y) {}
    }

    public static final class IntakeAssembly { // [ ] Intake Assembly constants
        public enum IntakeAssemblyState { 
            // 70 max angle upwards
            // TODO find values for the intake wirst and elevator
            GROUNDPICKUP(Rotation2d.fromDegrees(-30), 0.0), 
            STOW(Rotation2d.fromDegrees(90), 0.0), 
            AMP(Rotation2d.fromDegrees(-23), 0.36), //TODO find elevator height 
            Climb(Rotation2d.fromDegrees(-20), 4.3), 
            LOAD(Rotation2d.fromDegrees(65), 0.0), 
            SOURCE(Rotation2d.fromDegrees(80.0), 0.143993),  //TODO find elevator height 
            MANUAL(Rotation2d.fromDegrees(0.0), 0.0); // CHECKUP
                                                                                                             // is this
                                                                                                             // needed?

            public final Rotation2d wristAngle;
            public final double ElevatorHeight;

            private IntakeAssemblyState(Rotation2d wristAngle, double elevatorHeight) {
                this.wristAngle = wristAngle;
                ElevatorHeight = elevatorHeight;
            }

        }

        public static final class IntakeConstants {// [ ] Intake constants
            public static final MotorConstants kMotorConstants = new MotorConstants(15, MotorType.kBrushless, true,
                    IdleMode.kBrake, 60);
            public static final MotorConstants kFollowerMotorConstants = new MotorConstants(23, MotorType.kBrushless, true,
                IdleMode.kBrake, 60);

            public static final double kMainToFollowRatio = -1/2 * 6748.0/5676.0;

            public static final PIDConstants kPIDconstants = new PIDConstants(1.0, 0.0, 0.0); // HACK Tune, and test

            public static final double kInSpeed = 0.8; // placeholder
            public static final double kLoadSpeed = 0.5; // placeholder
            public static final double kOutSpeed = 0.5; // placeholder
            public static final double kOffSpeed = 0;

            public static final boolean kUseBeamBreak = true;
            public static final int kIntakeBeamBreakID = 0; // placeholder
        }

        public static final class IntakeWristConstants { // [ ] Intake Wrist constants

            // TODO Make Units Clear

            public static final double kLimitSwitchTriggerOffset = -0.025;

            public static final MotorConstants kMotorConstants = new MotorConstants(19, MotorType.kBrushless, false,
                    IdleMode.kBrake, 40);
            // public static final PIDConstants kPIDConstants = new PIDConstants(0.25, 0.0, 0.0); // TODO Tune
            // public static final Constraints kTrapzoidConstraints = new Constraints(1, 1); // TODO Tune

            public static final double kWristToRotationsRate = 44.7631578947;
            
            // public static final IntakeAssemblyState kStartupState = IntakeAssemblyState.STOW;

            public static final double kMeterSafetyLimit = 0.23; // HACK tested in sim

            public static final int kCANCoderID = 25;

            public static final Rotation2d kCanCoderResetAngle = Rotation2d.fromDegrees(15);

            public static final double kCANCoderOffset = 0.0;

            //TODO find values for this
            public static final Rotation2d kMaxAngleAmp = Rotation2d.fromDegrees(0); //only when above the safety height
            public static final Rotation2d kMaxAngleGround = Rotation2d.fromDegrees(91/*291*/); //only when below the safety height
            public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(-40); 

            public static final Rotation2d kStartingAngle = Rotation2d.fromDegrees(100.8);

            public static final PIDConstants kPIDconstants; // don't test with these values
            static{
                final double shooterRotationsNeedingFullPower = Rotation2d.fromDegrees(15).getRotations();
                final double motorRotationsNeedingFullPower = (shooterRotationsNeedingFullPower
                        * ShooterWristConstants.kWristToRotationsRate);
                final double maxMotorPowerSetting = 1;
                final double P = maxMotorPowerSetting / motorRotationsNeedingFullPower;
    
                kPIDconstants = new PIDConstants(P, 0.0, 0.0);
            }
    
            public static final Constraints kConstraints;
    
            static{
                 // The number of seconds that we expect the shooter to go from in to Max
                final double timeMinToMaxSeconds = 0.25;
                // How long we expect the shooter to take to get to full speed
                final double timeToFullSpeedSeconds = 0.05;
                final double maxShooterRotations = IntakeWristConstants.kMaxAngleGround.getRotations()
                        - ShooterWristConstants.kMinAngle.getRotations();
                final double maxWristVelocity = maxShooterRotations / timeMinToMaxSeconds;
                final double maxWristAcceleration = maxWristVelocity / timeToFullSpeedSeconds;
    
                kConstraints = new Constraints(maxWristVelocity, maxWristAcceleration);
            }

            public static final FeedForwardConstants kArmFeedForward = new FeedForwardConstants(1.0, 1.0, 1.0, 0.0); // HACK
                                                                                                                     // untested
                                                                                                                     // values

            public static final int kLimitSwitchChannel = 4;
            public static final double kLimitSwitchEncoderReading = 0;

            public static final class ManualConstants { // speed of manual movements,

            }
        }

        public static final class ElevatorConstants {
            private static final int kMotorLimit = 40;
            
            public static final MotorConstants kMotorConstants = new MotorConstants(20, MotorType.kBrushless, false,
                    IdleMode.kBrake, kMotorLimit);
            public static final MotorConstants kFollowerConstants = new MotorConstants(16, MotorType.kBrushless, false,
                    IdleMode.kBrake, kMotorLimit);
            
                    public static final double kMetersToRotation = 40 / 0.23; // Conversion rate
            
            public static final Constraints kZoidConstants = new Constraints(
                2.0, 
                5.0
            );

            public static final PIDConstants kPIDConstants = new PIDConstants(
                0.2/(0.05 * kMetersToRotation) /* */, 
            0, 0);

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
            public static final int kLimitSwitchChannel = 3;

            public static final double kLimitSwitchTriggerOffset = -0.025;

            public static final double kLimitSwitchGoto = 0; // Where the elevator will go to if the limit switch is
                                                             // triggered
            public static final double kMinMeters = 0.0;
            public static final double kMaxMeters = 0.38; // hack untested
        }
    }

    public static final class ShooterWristConstants { // [ ] Shooter Wrist Constants
        public enum ShooterWristState { // Mostly for debug
            SUBWOOFER_SHOT(Rotation2d.fromDegrees(56.6864193)), // TODO find Value
            HARD_STOP(Rotation2d.fromDegrees(14.9)), //TODO find value
            STOW(Rotation2d.fromDegrees(68));

            public final Rotation2d shooterAngle;

            private ShooterWristState(Rotation2d shooterAngle) {
                this.shooterAngle = shooterAngle;
            }
        }

        public static double kWristToRotationsRate = 66.5;

        public static final MotorConstants kMotorConstants = new MotorConstants(21, MotorType.kBrushless, false,
                IdleMode.kBrake, 0); // placeholder
        
        
    
        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(80); //only when above the safety height
        public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(15); 

        // public static final Rotation2d kStartingAngle = Rotation2d.fromDegrees(74);

        public static final FeedForwardConstants kWristFeedForward = new FeedForwardConstants(1.0, 1.0, 1.0, 0.0); // HACK
                                                                                                                   // untested
                                                                                                                   // values

        public static final double kAimedAtTargetThreshold = Rotation2d.fromDegrees(2).getRotations(); // 6 degrees

        public static final int kLimitSwitchChannel = 5;
        public static final double kLimitSwitchEncoderReading = Rotation2d.fromDegrees(70).getRotations();

        public static final double kLimitSwitchTriggerOffset = -0.025;

                
        public static final PIDConstants kPIDconstants; // don't test with these values
        static{
            final double shooterRotationsNeedingFullPower = Rotation2d.fromDegrees(15).getRotations();
            final double motorRotationsNeedingFullPower = (shooterRotationsNeedingFullPower
                    * ShooterWristConstants.kWristToRotationsRate);
            final double maxMotorPowerSetting = 1;
            final double P = maxMotorPowerSetting / motorRotationsNeedingFullPower;
            System.out.println(P);
            System.out.println(P);
            System.out.println(P);
            System.out.println(P);

            kPIDconstants = new PIDConstants(0.11, 0.0, 0.0);
        }

        public static final Constraints kConstraints;

        static {
             // The number of seconds that we expect the shooter to go from in to Max
            final double timeMinToMaxSeconds = 0.8;
            // How long we expect the shooter to take to get to full speed
            final double timeToFullSpeedSeconds = 0.2;
            final double maxShooterRotations = ShooterWristConstants.kMaxAngle.getRotations()
                    - ShooterWristConstants.kMinAngle.getRotations();
            final double maxWristVelocity = maxShooterRotations / timeMinToMaxSeconds;
            final double maxWristAcceleration = maxWristVelocity / timeToFullSpeedSeconds;

            kConstraints = new Constraints(maxWristVelocity, maxWristAcceleration);
        }

        
    }

    public static final class ShooterConstants { // [ ] Shooter Constants
        public static final MotorConstants kShooterMotorConstants = new MotorConstants(18, MotorType.kBrushless, false,
                IdleMode.kCoast, 40); // placeholder
        public static final MotorConstants kShooterFollowMotorConstants = new MotorConstants(22, MotorType.kBrushless,
                false, IdleMode.kCoast, 40); // placeholder
        public static final MotorConstants kConveyorMotorConstants = new MotorConstants(17, MotorType.kBrushless, false,
                IdleMode.kCoast, 60); // placeholder
        public static final PIDFConstants kPIDconstants = new PIDFConstants(0.6, 2.4, 0.0375, 0.0); // K_u = 1.0, T_u = 0.5
        public static final double kOffSpeed = 0.0; // unsure if this is necessary
        public static final double kShootSpeed = 5600; // placeholder
        public static final double kDiff = 200;
        public static final double kConveyorInSpeed = 0.5; // placeholder
        public static final double kConveyorOutSpeed = 0.8; // placeholder

        public static final double kTargetRPM = 3500;
    }

    public static final class AutoAimConstants {
        public enum Targets {
            TEMP_TARGET(new Translation3d(0.2286, 5.5, 2.0)), BLUE_SPEAKER(
                    new Translation3d(0.2286, 5.5, 2.0)), RED_SPEAKER(new Translation3d(0.2286, 5.5, 2.0)); // GET RED
                                                                                                            // SPEAKER
                                                                                                            // POSITION

            public final Translation3d targetPosition;

            private Targets(Translation3d targetPosition) {
                this.targetPosition = targetPosition;
            }
        }

        public static final Translation3d kSpeakerTarget;
        static {
            if (DriverStation.getAlliance().isPresent()) {
                switch (DriverStation.getAlliance().get()) {
                    case Blue:
                        kSpeakerTarget = Targets.BLUE_SPEAKER.targetPosition;
                        break;
                    case Red:
                        kSpeakerTarget = Targets.RED_SPEAKER.targetPosition;
                        break;
                    default:
                        kSpeakerTarget = Targets.BLUE_SPEAKER.targetPosition;
                        break;
                }
            } else {

                kSpeakerTarget = Targets.TEMP_TARGET.targetPosition; 
            }
        }

        public static final Translation3d kAutoAimTarget = new Translation3d(0.2286, 5.5, 2.0);
        // public static final Translation3d kAutoAimTarget = new Translation3d(5, 5, 0.1);
        public static final double kFlyWheelTransferRate = 0.9; //CHECKUP kinda a guess
        public static final double kFlyWheelRadPerSec = (kTargetRPM/*rpm, curr free speed */)*(2*Math.PI)*(1/60.0);
        public static final double kFlyWheelRadius = 0.038;
        public static final double kShooterSpeed = kFlyWheelTransferRate * kFlyWheelRadius * kFlyWheelRadPerSec; // needs to be in m/s
        public static final double kShooterHeight = 0.1681988;
        public static final double kMaxDistance = 10.0; // Needs units, the maximum relative distance a target can be
                                                        // from the robot for autoaim
        public static final double kGravity = -9.8;
        public static final int kIterations = 45;
    }

    public static final class Vision {
        public enum VisionPipelines {
            FIDUCIALS_3D(0, false), DETECTOR_NOTE(1, true);

            public final int pipeline;
            public final boolean isDetector;

            private VisionPipelines(int pipeline, boolean isDetector) {
                this.pipeline = pipeline;
                this.isDetector = isDetector;
            }
        }

        public static final class PoseOffsetConstants {
            public static final Transform3d kAlicePoseOffset = new Transform3d(); // new Transform3d(-0.22, -0.1, 0, new
                                                                                  // Rotation3d(0, 0, 0)); //
                                                                                  // placeholder
            public static final Transform3d kBobPoseOffset = new Transform3d(); // new Transform3d(-0.35, -0.1, 0, new
                                                                                // Rotation3d(0, 0,
                                                                                // Units.degreesToRadians(180))); //
                                                                                // placeholder
        }
    }

    public static class BlingConstants {
        public static final int kLedPort = 0; // Port that light strip is on
        public static final int kLedLength = 150; // Length of light strip
        public static final int kAlertLength = 60; // Pulse Length in Frames
        public static final int kPulseLength = 200; // Pulse Length in Frames
        public static final double kAroundSpeedMultiplier = .2; // Around speed multiplier
        public static final double kAroundStripLength = 15; // Around speed multiplier
        public static final double kBrightness = 1; // Percentage
        public static final BlingModes kDefaultBlingMode = BlingModes.AROUND_SECONDARY_BG;
        public static final Color kDefaultBlingColor = Color.kGold;
        public static final Color kDefaultBlingColorSecondary = Color.kRoyalBlue;
        public static final boolean kIsInFancyMode = true;
        public static final boolean kSpam = false;
    }

    public enum BlingModes {
        OFF, SOLID, SOLID_SECONDARY, GRADIENT, GRADIENT_REVERSED, PULSE, PULSE_SWITCH, AROUND, AROUND_SECONDARY_BG, WARNING, ERROR
    }
}
