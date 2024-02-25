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
import com.spartronics4915.frc2024.util.ModeSwitchInterface;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.spartronics4915.frc2024.Constants.IntakeAssembly.ElevatorConstants.*;

public class Elevator extends SubsystemBase implements TrapezoidSimulatorInterface, ModeSwitchInterface {
    //#region all the variables and stuff
    private static Elevator mInstance;

    private CANSparkMax mMotor;
    private CANSparkMax mFollower;
    private SparkPIDController mPid;
    private RelativeEncoder mEncoder;

    private TrapezoidProfile mTrapezoid;

    private State mCurrentState;
    private double mTarget;// = new Rotation2d(Math.PI * 3);
    private double mManualDelta;

    private ElevatorFeedforward mElevatorFeedforward;

    private boolean mIsManual = false;

    private GenericEntry mElevatorSetPointEntry;
    private GenericEntry mElevatorHeightEntry;
    private GenericEntry mElevatorManualControlEntry;
    private GenericEntry mElevatorLeaderPos;
    private GenericEntry mElevatorFollowerPos;
    private GenericEntry mAppliedOutputWidget;

    private DigitalInput limitSwitch;

    private boolean startupHome = false;
    private boolean mHoming = false;

    private RelativeEncoder mFollowerEncoder;


    // #endregion

    public Elevator() {
        // Initializes the motor
        mMotor = new CANSparkMax(kMotorConstants.motorID(), kMotorConstants.motorType());
        mMotor.restoreFactoryDefaults();
        mMotor.setInverted(kMotorConstants.motorIsInverted());
        mMotor.setIdleMode(kMotorConstants.idleMode());
        mMotor.setSmartCurrentLimit(kMotorConstants.currentLimit());

        // Initializes the follower
        mFollower = new CANSparkMax(kFollowerConstants.motorID(), kFollowerConstants.motorType());
        mFollower.restoreFactoryDefaults();
        mFollower.setInverted(kFollowerConstants.motorIsInverted());
        mFollower.setIdleMode(kFollowerConstants.idleMode());
        mFollower.setSmartCurrentLimit(kFollowerConstants.currentLimit());
        mFollower.follow(mMotor);

        // Initializes the Trapezoid
        mTrapezoid = new TrapezoidProfile(kZoidConstants);

        // Initializes the PID
        mPid = mMotor.getPIDController();
        mPid.setP(kPIDConstants.p());
        mPid.setI(kPIDConstants.i());
        mPid.setD(kPIDConstants.d());

        mMotor.burnFlash();
        // CHECKUP Decide on Vel conversion Factor (aka use rpm?)

        // Sets up the encoder
        mEncoder = mMotor.getEncoder();
        mFollowerEncoder = mFollower.getEncoder(); //TODO remove this and the print lines

        // Set Encoders to 0 just for initialization
        mEncoder.setPosition(0);
        mFollower.getEncoder().setPosition(0);

        // Sets up Feed Foward
        mElevatorFeedforward = new ElevatorFeedforward(kElevatorFeedFowardConstants.kS(),
                kElevatorFeedFowardConstants.kG(), kElevatorFeedFowardConstants.kV());

        // Sets the current state and target
        resetTarget();

        // Sets up the Limit Switch
        limitSwitch = new DigitalInput(kLimitSwitchChannel);

        new Trigger(limitSwitch::get).onTrue(new Command() {

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }

            @Override
            public void execute() {
                mEncoder.setPosition(kLimitSwitchGoto*kMetersToRotation);
                // if (mTarget.getRotations() < kLimitSwitchGoto * kMetersToRotation + kLimitSwitchTriggerOffset) { //CHECKUP does trigger get hit rapidly
                    mTarget = (kLimitSwitchGoto);
                    updateCurrStateToReal();
                // }
                startupHome = true;
                mHoming = false;
                mIsManual = false;
            }
            
            @Override
            public boolean isFinished() {
                return true;
            }
        });


        initShuffle();
    }

    @Override
    public void periodic() {
        if (mIsManual) { // Manual
            manualControlUpdate();
            if (!mHoming) mTarget = (Math.max(mTarget, kMinimumManualRotations));
        }
        // Not-manual
        // switching with trigger
        // if (limitSwitch.get() && Robot.isReal()){
        //     mEncoder.setPosition(kLimitSwitchGoto * kMetersToRotation);
        //     if (mTarget.getRotations() < kLimitSwitchGoto * kMetersToRotation + kLimitSwitchTriggerOffset ) {
                
        //         mTarget = Rotation2d.fromRotations(kLimitSwitchGoto * kMetersToRotation);
        //     }
            
        // }
        if (!mHoming) {
            mTarget = MathUtil.clamp(mTarget, kMinimumManualRotations, kMaximumRotations);

            mTarget = Math.max(mTarget, 0);
        }
        
        mCurrentState = mTrapezoid.calculate(
                GeneralConstants.kUpdateTime,
                mCurrentState,
                // new State(getEncoderPosReading().getRotations(), getEncoderVelReading()),
                new State(mTarget, 0));
        // mPid.setReference(mCurrentState.position * kMetersToRotation, ControlType.kPosition, 0, getFeedForwardValue());
        System.out.println(mEncoder.getPosition());
        System.out.println(mFollowerEncoder.getPosition());
        updateShuffle();
    }

    // #region encoder & feed foward
    private double getEncoderVelReading() {
        return mEncoder.getVelocity(); // CHECKUP Failure Point?
    }

    private double getEncoderPosReading() {
        return (mEncoder.getPosition()); // CHECKUP Failure Point?
    }

    private double getFeedForwardValue() {
        return 0;
        //return mElevatorFeedforward.calculate(getEncoderVelReading());
    }
    // #endregion

    // #region Shuffleboard
    private void initShuffle() {
        var mEntries = ElevatorTabManager.getEnumMap(this);
        mElevatorSetPointEntry = mEntries.get(ElevatorSubsystemEntries.ElevatorSetPoint);
        mElevatorHeightEntry = mEntries.get(ElevatorSubsystemEntries.ElevatorHeight);
        mElevatorManualControlEntry = mEntries.get(ElevatorSubsystemEntries.ElevatorManualControl);
        mElevatorLeaderPos = mEntries.get(ElevatorSubsystemEntries.ElevatorLeaderPos);
        mElevatorFollowerPos = mEntries.get(ElevatorSubsystemEntries.ElevatorFollowerPos);
        mAppliedOutputWidget = mEntries.get(ElevatorSubsystemEntries.ElevatorLeaderAppliedOutput);
    }

    private void updateShuffle() {
        mElevatorSetPointEntry.setDouble(mTarget);
        mElevatorHeightEntry.setDouble(getHeight());
        mElevatorManualControlEntry.setBoolean(mIsManual);
        mElevatorLeaderPos.setDouble(mEncoder.getPosition());
        mElevatorFollowerPos.setDouble(mFollower.getEncoder().getPosition());
        mAppliedOutputWidget.setDouble(mMotor.getAppliedOutput());
    }
    // #endregion

    // #region Bunch of random getters
    @Override
    public State getSimulatedSetPoint() {
        return new State(mCurrentState.position, 0.0);
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
        return getEncoderPosReading();
    }

    /**
     * @return height of elevator
     */
    public double getDistance() {
        return getHeight();
    }

    // #endregion

    // #region Maunel Manuel Manueal Manael Manual Stuff (5th times the charm)

    private void manualControlUpdate() {
        mTarget = (mTarget + mManualDelta);
    }

    private void setManualDelta(double deltaPosition) {
        mIsManual = true;
        mManualDelta = deltaPosition;
    }

    private void homeMotor(double angleDelta){
        mHoming = true;
        setManualDelta(angleDelta);
    }

    /**
     * Command for manual movement
     * @param angleDelta
     */
    public Command manualRunCommand(double posDelta) {
        return this.startEnd(() -> setManualDelta(posDelta), () -> {
            if (!Robot.isSimulation())
                resetTarget();
            mIsManual = false;
        });
    }

    public Command homeMotorCommand(double angleDelta){
        return runOnce(() -> {
            homeMotor(angleDelta);
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
        mTarget = (newTarget * kMetersToRotation);
    }

    /**
     * Sets the new target position
     * @param intakeAssemblyState I don't know what this is but I was told to add it so I did
     */
    public void setTarget(IntakeAssemblyState intakeAssemblyState) {
        mIsManual = false;
        mTarget = (intakeAssemblyState.ElevatorHeight * kMetersToRotation);
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

    public boolean atTargetState(double rotationThreshold){
        return (Math.abs(getEncoderPosReading() - mTarget) < rotationThreshold);
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
        updateCurrStateToReal();
        mTarget = getEncoderPosReading();
    }

    private void updateCurrStateToReal(){
        this.mCurrentState = new State(getEncoderPosReading() / kMetersToRotation, 0);
    }

    // #endregion

    // #region i'm just ignoring these things

        // Here because something requires it
        @Override
        public void modeSwitchAction() {}

    /**
     * @return A static instance of the elevator subsystem
     */
    public static Elevator getInstance() {
        if (mInstance == null)
            mInstance = new Elevator();
        return mInstance;
    }

    /**
     * If you ever need to not get an instance of the elevator subsystem, use this
     * @return null
     */
    public static Elevator dontGetInstance() {
        return null;
    }

    /**
     * getInstance for gambling addicts. 50% of the time you will get the instance but the other 50% you won't
     * @return A static instance of the elevator subsystem OR null
     */
    public static Elevator mightGetInstance() {
        if (Math.random() < .2) // Actually less than 50% because the house always wins
            return getInstance();
        return null;
    }

    // #endregion

}
