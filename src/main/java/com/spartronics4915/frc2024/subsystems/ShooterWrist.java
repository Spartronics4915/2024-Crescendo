package com.spartronics4915.frc2024.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.spartronics4915.frc2024.Constants.ShooterWristConstants;
import com.spartronics4915.frc2024.Constants.GeneralConstants;
import com.spartronics4915.frc2024.Constants.ShooterConstants;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeWristConstants;
import com.spartronics4915.frc2024.Constants.ShooterWristConstants.ShooterWristState;
import com.spartronics4915.frc2024.ShuffleBoard.ShooterWristTabManager;
import com.spartronics4915.frc2024.ShuffleBoard.ShooterWristTabManager.ShooterWristSubsystemEntries;
import com.spartronics4915.frc2024.Robot;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.SimType;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.SimulatorSettings;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.TrapezoidSimulatorInterface;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;
import com.spartronics4915.frc2024.util.ModeSwitchInterface;
import com.spartronics4915.frc2024.util.MotorConstants;
import com.spartronics4915.frc2024.util.PIDConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import java.util.Set;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.spartronics4915.frc2024.Constants.ShooterWristConstants.*;
import java.util.function.*;

import org.ejml.dense.row.decomposition.hessenberg.TridiagonalDecomposition_FDRB_to_FDRM;

public class ShooterWrist extends SubsystemBase implements TrapezoidSimulatorInterface, ModeSwitchInterface {

    // #region variables

    private CANSparkMax mWristMotor;
    private SparkPIDController mPidController;

    private RelativeEncoder mEncoder;
    private TrapezoidProfile kTrapezoidProfile;

    private State mCurrentState;
    private Rotation2d mTargetRotation2d;
    private Rotation2d mManualDelta = new Rotation2d();
    private final ArmFeedforward kFeedforwardCalc;

    private static ShooterWrist mInstance;

    private boolean mManualMovement = false; // used to pause position setting to avoid conflict (if using Trapezoid
                                             // movment due to the constant calls)

    private DigitalInput mLimitSwitch;

    private GenericEntry mShooterSetPointEntry;
    private GenericEntry mShooterEncoderReadingEntry;
    private GenericEntry mShooterManualControlEntry;
    private GenericEntry mShooterDelta;
    private GenericEntry mAppliedOutput;
    private GenericEntry mShooterWristErrorPID;
    private GenericEntry mShooterWristErrorTrapazoid;

    private SendableChooser<Rotation2d> chooser;

    private boolean startupHome = false;
    private boolean mHoming = false;

    // #endregion

    public static ShooterWrist getInstance() {
        if (mInstance == null) {
            mInstance = new ShooterWrist();
        }
        return mInstance;
    }

    public ShooterWrist() {
        super();
        mWristMotor = initMotor(ShooterWristConstants.kMotorConstants);
        mPidController = initPID();
        mEncoder = initEncoder();
        kTrapezoidProfile = initTrapezoid();
        mLimitSwitch = initLimitSwitch();

        kFeedforwardCalc = initFeedForward();

        resetEncoder(kStartingAngle);

        mWristMotor.burnFlash();

        currentToSetPoint();

        initShuffleBoard();

        new Trigger(mLimitSwitch::get).onTrue(new Command() {

            @Override
            public boolean runsWhenDisabled() {
                return false;
            }

            @Override
            public void execute() {
                mEncoder.setPosition(kLimitSwitchEncoderReading * kWristToRotationsRate);
                // if (mTargetRotation2d.getRotations() < kLimitSwitchEncoderReading * kInToOutRotations +
                // kLimitSwitchTriggerOffset) { //CHECKUP does trigger get hit rapidly
                currentToSetPoint(Rotation2d.fromRotations(kLimitSwitchEncoderReading));
                // }
                mHoming = false;
                mManualMovement = false;
                startupHome = true;
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        });
    }

    // #region init functions

    private void initShuffleBoard() {
        var mEntries = ShooterWristTabManager.getEnumMap(this);
        ShooterWristTabManager.addMotorControlWidget(this);
        
        mShooterSetPointEntry = mEntries.get(ShooterWristSubsystemEntries.ShooterSetPoint);
        mShooterEncoderReadingEntry = mEntries.get(ShooterWristSubsystemEntries.ShooterEncoderReading);
        mShooterManualControlEntry = mEntries.get(ShooterWristSubsystemEntries.ShooterManualControl);
        mShooterDelta = mEntries.get(ShooterWristSubsystemEntries.ShooterDelta);
        mAppliedOutput = mEntries.get(ShooterWristSubsystemEntries.WristAppliedOutput);

        mShooterWristErrorPID = mEntries.get(ShooterWristSubsystemEntries.ShooterWristErrorPID);
        mShooterWristErrorTrapazoid = mEntries.get(ShooterWristSubsystemEntries.ShooterWristErrorTrapazoid);

        chooser = new SendableChooser<Rotation2d>();

        chooser.addOption("MIN", kMinAngle);
        chooser.addOption(""+ShooterWristState.SUBWOOFER_SHOT.shooterAngle.getDegrees(), ShooterWristState.SUBWOOFER_SHOT.shooterAngle);
        chooser.addOption(""+ShooterWristState.STOW.shooterAngle.getDegrees(), ShooterWristState.STOW.shooterAngle);
        chooser.addOption("MAX", kMaxAngle);


        SmartDashboard.putData("choose shooterWrist value", chooser);
    }

    private CANSparkMax initMotor(MotorConstants motorValues) {
        CANSparkMax motor = new CANSparkMax(motorValues.motorID(), motorValues.motorType());
        motor.restoreFactoryDefaults();
        motor.setInverted(motorValues.motorIsInverted());
        motor.setIdleMode(motorValues.idleMode());
        motor.setSmartCurrentLimit(motorValues.currentLimit());
        return motor;
    }

    private SparkPIDController initPID() {
        SparkPIDController pid = mWristMotor.getPIDController();

        pid.setOutputRange(-0.2, 0.2);

        pid.setP(kPIDconstants.p());
        pid.setI(kPIDconstants.i());
        pid.setD(kPIDconstants.d());

        // CHECKUP Decide on Vel conversion Factor (aka use rpm?)
        // position Conversion not needed by using rotation2d

        return pid;
    }

    private RelativeEncoder initEncoder() {

        RelativeEncoder encoder = mWristMotor.getEncoder();
        // Set the encoder to zero for now to make sure it is initialized to a reasonable value.
        encoder.setPosition(0); //TODO set to startup position
        return encoder;
    }

    private TrapezoidProfile initTrapezoid() {
        return new TrapezoidProfile(kConstraints);
    }

    private ArmFeedforward initFeedForward() {
        var out = new ArmFeedforward(kWristFeedForward.kS(), kWristFeedForward.kG(), kWristFeedForward.kV(),
                kWristFeedForward.kA());

        return out;
    }

    private DigitalInput initLimitSwitch() {
        DigitalInput out = new DigitalInput(kLimitSwitchChannel);
        return out;
    }

    // #endregion

    // #region component fucntions

    private double getEncoderVelReading() {
        return mEncoder.getVelocity(); // CHECKUP Failure Point?
    }

    private void setManualDelta(Rotation2d deltaPosition) {
        mHoming = false;
        mManualMovement = true;
        mManualDelta = deltaPosition;
    }

    private void homeMotor(Rotation2d deltaPosition) {
        mHoming = true;
        setManualDelta(deltaPosition);
    }

    private void setRotationSetPoint(Rotation2d angle) {
        mTargetRotation2d = angle;
    }

    public void publicSetRotationSetPoint(Rotation2d angle) {
        setRotationSetPoint(angle);
    }

    public Rotation2d getWristAngle() {
        return Rotation2d.fromRotations(mEncoder.getPosition()).div(kWristToRotationsRate);
    }

    private void setState(ShooterWristState newState) {
        mManualMovement = false;
        publicSetRotationSetPoint(newState.shooterAngle);
    }

    private void currentToSetPoint() {
        currentToSetPoint(getWristAngle());
    }

    private void currentToSetPoint(Rotation2d setpoint) {
        updateCurrStateToReal(setpoint);
        System.out.println(setpoint);
        setRotationSetPoint(setpoint); 
    }

    private void updateCurrStateToReal(Rotation2d setpoint) {
        mCurrentState = new State(setpoint.getRotations(), 0.0);
    }

    public boolean atTarget() {
        return Math.abs(getWristAngle().getRotations() - mTargetRotation2d.getRotations()) < kAimedAtTargetThreshold;
    }

    // #endregion

    // #region Commands

    public Command resetEncoder() {
        return resetToAngle(0.0);
    }

    public Command resetToAngle(double degrees) {
        return runOnce(() -> {
            resetEncoder(Rotation2d.fromDegrees(degrees));
        });
    }

    public Command setPidConstant(PIDConstants values){
        return runOnce(() -> {
            mPidController.setP(values.p());
            mPidController.setI(values.i());
            mPidController.setD(values.d());
        });
    }

    public void resetEncoder(Rotation2d angle){
        mEncoder.setPosition(angle.getRotations() * kWristToRotationsRate);
        currentToSetPoint(angle);
    }

    public Command angleToSupplierCommand(Supplier<Rotation2d> supplier) {
        return Commands.run(() -> {
            setRotationSetPoint(supplier.get());
        }, this);
    }

    public Command setValueFromChooser(){
        return Commands.runOnce(() -> {
            setRotationSetPoint(chooser.getSelected());
        });
    }

    public Command setStateCommand(ShooterWristState newState) {
        return Commands.runOnce(() -> {
            setState(newState);
        });
    }

    public Command manualRunCommand(Rotation2d angleDelta) {
        return this.startEnd(
                () -> setManualDelta(angleDelta),
                () -> {
                    if (!Robot.isSimulation())
                        currentToSetPoint(); // CHECKUP remove sim statment when doing chassis stuff?
                    mManualMovement = false;
                });
    }

    public Command homeMotorCommand(Rotation2d angleDelta) {
        return runOnce(() -> {
            homeMotor(angleDelta);
        });
    }

    // #endregion

    // #region periodic functions

    @Override
    public void periodic() {
        if (mManualMovement) {
            manualControlUpdate();
        }
        TrapezoidMotionProfileUpdate();

        updateShuffle();
        handleLimitSwitch();
        // will add things here if trapezoid motion profiles get used
    }

    private void updateShuffle() {
        mShooterSetPointEntry.setDouble(mTargetRotation2d.getDegrees());
        mShooterEncoderReadingEntry.setDouble(getWristAngle().getDegrees());
        mShooterManualControlEntry.setBoolean(mManualMovement);
        mShooterDelta.setDouble(mManualDelta.getDegrees());
        mAppliedOutput.setDouble(mWristMotor.getAppliedOutput());
        
    }

    private double getFeedForwardValue() {
        return kFeedforwardCalc.calculate(
                getWristAngle().getRadians() / kWristToRotationsRate,
                (getEncoderVelReading() / 60.0) * 2 * Math.PI // convert from RPM --> Rads/s
        );
        // return 0.0;
    }

    private void manualControlUpdate() { // HACK untested
        // if (!mHoming) {
        //     if (mTargetRotation2d.getRotations() % 1.0 > kMaxAngle.getRotations() && mManualDelta.getRotations() > 0) { // CHECKUP
        //                                                                                                                 // might
        //                                                                                                                 // not
        //                                                                                                                 // work
        //         System.out.println(mTargetRotation2d.getDegrees() % 1.0);
        //         System.out.println(kMaxAngle.getRotations());
        //         System.out.println("reset max");
        //         mTargetRotation2d = kMaxAngle;
        //     } else if (mTargetRotation2d.getRotations() % 1.0 < kMinAngle.getRotations()
        //             && mManualDelta.getRotations() < 0) {
        //         System.out.println(mTargetRotation2d.getDegrees() % 1.0);
        //         System.out.println(kMinAngle.getRotations());
        //         System.out.println("reset low");
        //         mTargetRotation2d = kMinAngle;
        //     } else {
                mTargetRotation2d = mTargetRotation2d.plus(mManualDelta);
        //     }
        // }
    }

    private void TrapezoidMotionProfileUpdate() {
        if (!mHoming)
            mTargetRotation2d = Rotation2d.fromRotations(
                MathUtil.clamp(mTargetRotation2d.getRotations(), kMinAngle.getRotations(), kMaxAngle.getRotations())
            );

        // CHECKUP not sure if this will work
        mCurrentState = kTrapezoidProfile.calculate(
                GeneralConstants.kUpdateTime,
                mCurrentState,
                new State(mTargetRotation2d.getRotations(), 0));

        mShooterWristErrorPID.setDouble(Rotation2d.fromRotations(mCurrentState.position).getDegrees() - getWristAngle().getDegrees());
        mShooterWristErrorTrapazoid.setDouble(mTargetRotation2d.getDegrees() - Rotation2d.fromRotations(mCurrentState.position).getDegrees());

        mPidController.setReference(mCurrentState.position * kWristToRotationsRate, ControlType.kPosition, 0, 0);
        // CHECKUP FF output? currently set to volatgage out instead of precentage out
    }

    private void handleLimitSwitch() {
        // switching with triggers
        // if (mLimitSwitch.get()) {
        // mEncoder.setPosition(kLimitSwitchEncoderReading*kInToOutRotations);
        // if (mTargetRotation2d.getRotations() < kLimitSwitchEncoderReading * kInToOutRotations +
        // kLimitSwitchTriggerOffset) {
        // mTargetRotation2d = Rotation2d.fromRotations(kLimitSwitchEncoderReading * kInToOutRotations);
        // }
        // }
    }

    // #endregion
    @Override
    public void modeSwitchAction() {
        currentToSetPoint();
    }

    @Override
    public State getSimulatedSetPoint() {
        return new State(0.5 - mCurrentState.position, 0.0);
    }

    @Override
    public SimulatorSettings getSettings() {
        return new SimulatorSettings(
                "Shooter",
                1.0,
                0.0,
                5.0,
                new Color8Bit(Color.kDarkGreen),
                SimType.Angle,
                new Translation2d(2, 0));
    }

}
