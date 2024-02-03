package com.spartronics4915.frc2024.subsystems;

import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.ShuffleBoard.ElevatorTabManager;
import com.spartronics4915.frc2024.ShuffleBoard.ElevatorTabManager.ElevatorSubsystemEntries;
import com.spartronics4915.frc2024.Robot;
import com.spartronics4915.frc2024.Constants.GeneralConstants;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.SimulatorSettings;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.TrapezoidSimulatorInterface;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.spartronics4915.frc2024.util.TrapezoidSubsystemInterface;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2024.Constants.IntakeAssembly.ElevatorConstants.*;

public class Elevator extends SubsystemBase implements TrapezoidSimulatorInterface, TrapezoidSubsystemInterface {
    // #region all the variables and stuff
    private static Elevator mInstance;

    private CANSparkMax mMotor;
    private SparkPIDController mPid;
    private RelativeEncoder mEncoder;

    private TrapezoidProfile mmmmmmmmmmTrapezoid;

    private State mCurrentState;
    private Rotation2d mTarget;// = new Rotation2d(Math.PI * 3);
    private Rotation2d mManualDelta;

    private ElevatorFeedforward mElevatorFeedforward;

    private boolean mIsManual = false;

    private GenericEntry mElevatorSetPointEntry;
    private GenericEntry mElevatorHeightEntry;
    private GenericEntry mElevatorManualControlEntry;
    // #endregion

    public Elevator() {
        // Initializes the motor
        mMotor = new CANSparkMax(kMotorConstants.motorID(), kMotorConstants.motorType());
        mMotor.restoreFactoryDefaults();
        mMotor.setInverted(kMotorConstants.motorIsInverted());
        mMotor.setIdleMode(kMotorConstants.idleMode());
        mMotor.setSmartCurrentLimit(kMotorConstants.currentLimit());

        mMotor.burnFlash();
        // Initializes the Trapezoid
        mmmmmmmmmmTrapezoid = new TrapezoidProfile(kZoidConstants);

        // Initializes the PID
        mPid = mMotor.getPIDController();
        mPid.setP(0);
        mPid.setI(0);
        mPid.setD(0);
        // CHECKUP Decide on Vel conversion Factor (aka use rpm?)

        // Sets up the encoder
        mEncoder = mMotor.getEncoder();

        // Sets up Feed Foward
        mElevatorFeedforward = new ElevatorFeedforward(kElevatorFeedFowardConstants.kS(),
                kElevatorFeedFowardConstants.kG(), kElevatorFeedFowardConstants.kV());

        // Sets the current state and target
        resetTarget();

        initShuffle();
    }

    private void initShuffle() {
        var mEntries = ElevatorTabManager.getEnumMap(this);
        mElevatorSetPointEntry = mEntries.get(ElevatorSubsystemEntries.ElevatorSetPoint);
        mElevatorHeightEntry = mEntries.get(ElevatorSubsystemEntries.ElevatorHeight);
        mElevatorManualControlEntry = mEntries.get(ElevatorSubsystemEntries.ElevatorManualControl);
    }

    // #region encoder & feed foward
    private double getEncoderVelReading() {
        return mEncoder.getVelocity(); // CHECKUP Failure Point?
    }

    private Rotation2d getEncoderPosReading() {
        return Rotation2d.fromRotations(mEncoder.getPosition()); // CHECKUP Failure Point?
    }

    private double getFeedFowardValue() {
        return mElevatorFeedforward.calculate(getEncoderVelReading());
    }
    // #endregion

    @Override
    public void periodic() {
        if (mIsManual) { // Manual
            manualControlUpdate();
        }
        // Not-manual
        mCurrentState = mmmmmmmmmmTrapezoid.calculate(
                GeneralConstants.kUpdateTime,
                mCurrentState,
                // new State(getEncoderPosReading().getRotations(), getEncoderVelReading()),
                new State(mTarget.getRotations(), 0));
        mPid.setReference(mCurrentState.position, ControlType.kPosition, 0, getFeedFowardValue());

        updateShuffle();
    }

    private void updateShuffle() {
        mElevatorSetPointEntry.setDouble(mTarget.getDegrees() / kMetersToRotation);
        mElevatorHeightEntry.setDouble(getHeight());
        mElevatorManualControlEntry.setBoolean(mIsManual);
    }

    @Override
    public State getSetPoint() {
        return new State(mCurrentState.position / kMetersToRotation, 0.0);
    }

    /**
     * @return Simulator Settings for the Elevator
     */
    @Override
    public SimulatorSettings getSettings() {
        return kElevatorSimulatorSettings;
    }

    /**
     * @return height of elevator
     */
    public double getHeight() {
        return getEncoderPosReading().getRotations() / kMetersToRotation;
    }

    /**
     * @return height of elevator
     */
    public double getDistance() {
        return getHeight();
    }

    // #region Maunel Manuel Manueal Manael Manual Stuff (5th times the charm)

    private void manualControlUpdate() {
        mTarget = Rotation2d.fromRadians(mTarget.getRadians() + mManualDelta.getRadians());
    }

    private void setManualDelta(Rotation2d deltaPosition) {
        mIsManual = true;
        mManualDelta = deltaPosition;
    }

    /**
     * Command for manual movement
     * @param angleDelta
     */
    public Command manualRunCommand(Rotation2d angleDelta) {
        return this.startEnd(() -> setManualDelta(angleDelta), () -> {
            if (!Robot.isSimulation())
                resetTarget();
            mIsManual = false;
        });
    }

    // #endregion

    // #region Target (not the store)

    /**
     * Sets the new target position
     * @param newTarget New target position (in meters... i think)
     */
    public void setTarget(double newTarget) {
        mIsManual = false;
        mTarget = Rotation2d.fromRotations(newTarget * kMetersToRotation);
    }

    /**
     * Sets the new target position
     * @param intakeAssemblyState I don't know what this is but I was told to add it so I did
     */
    public void setTarget(IntakeAssemblyState intakeAssemblyState) {
        mIsManual = false;
        mTarget = Rotation2d.fromRotations(intakeAssemblyState.ElevatorHeight * kMetersToRotation);
    }

    /**
     * Command to set the new target position
     * @param newTarget New target position (in meters... i think)
     */
    public Command setTargetCommand(double newTarget) {
        return runOnce(() -> {
            setTarget(newTarget);
        });
    }
    
    /**
     * Command to set the new target position
     * @param intakeAssemblyState Whatever the intake assembly state
     */
    public Command setTargetCommand(IntakeAssemblyState intakeAssemblyState) {
        return runOnce(() -> {
            setTarget(intakeAssemblyState);
        });
    }

    /**
     * Sets the target position to the motor's current position so nobody gets punched in the face
     */
    public void resetTarget() {
        this.mCurrentState = new State(getEncoderPosReading().getRotations(), 0);
        mTarget = getEncoderPosReading();
    }

    // #endregion

    // #region i'm just ignoring these things

    // Here because something requires it
    @Override
    public void setPositionToReal() {}

    /**
     * @return A static instance of the elevator subsystem
     */
    public static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    /**
     * If you ever need to not get an instance of the elevator subsystem, use this
     * @return null
     */
    public static Elevator dontGetInstance() {
        return null;
    }

    // #endregion

}
