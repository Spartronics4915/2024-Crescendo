package com.spartronics4915.frc2024.subsystems.IntakeAssembly;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.spartronics4915.frc2024.util.MotorConstants;
import com.spartronics4915.frc2024.util.PIDConstants;
import com.spartronics4915.frc2024.RobotContainer;
import com.spartronics4915.frc2024.ShuffleBoard.IntakeTabManager;
import com.spartronics4915.frc2024.ShuffleBoard.IntakeTabManager.IntakeSubsystemEntries;
import com.spartronics4915.frc2024.util.Loggable;
import com.spartronics4915.frc2024.util.ModeSwitchInterface;

import static com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeConstants.*;

public class Intake extends SubsystemBase implements Loggable, ModeSwitchInterface {
    public static enum IntakeState {
        IN, LOAD, OUT, OFF, MANUAL, NONE; // NONE is only here as the Shuffleboard default value for troubleshooting
    }

    private static Intake mInstance;

    private IntakeState mCurrentState;

    private GenericEntry mIntakeStateWidget;
    private GenericEntry mManualSetPointWidget;
    private GenericEntry mIntakeVelocity;
    private GenericEntry mIntakeMotorCurrent;


    private final CANSparkMax mMotor;
    private final CANSparkFlex mFollowerMotor;

    private final SparkPIDController mPIDController;
    private final SparkPIDController mFollowPIDController;


    private final DigitalInput mBeamBreak;
    private final Timer mBeamBreakTimer;
    private final BooleanTopic mBeamBreakTopic;
    private final BooleanPublisher mBeamBreakPublisher;

    private double manualSetPoint;
    private final RelativeEncoder mEncoder;

    private Intake() {
        mCurrentState = IntakeState.OFF;

        var EntryMap = IntakeTabManager.getEnumMap(this);
        mIntakeStateWidget = EntryMap.get(IntakeSubsystemEntries.IntakeState);
        mManualSetPointWidget = EntryMap.get(IntakeSubsystemEntries.IntakeManualSetPoint);
        mIntakeVelocity = EntryMap.get(IntakeSubsystemEntries.IntakeVelocity);
        mIntakeMotorCurrent = EntryMap.get(IntakeSubsystemEntries.IntakeMotorCurrent);

        IntakeTabManager.addMotorControlWidget(this);

        //motor setup 

        mMotor = constructMaxMotor(kMotorConstants);
        mFollowerMotor = constructFlexMotor(kFollowerMotorConstants);

        //PID controller setup
        mPIDController = constructPIDController(mMotor, kPIDconstants);
        mFollowPIDController = constructPIDController(mFollowerMotor, kPIDconstants);

        mEncoder = mMotor.getEncoder();
        mMotor.burnFlash();

        //beam break setup
        mBeamBreak = new DigitalInput(kIntakeBeamBreakID);
        mBeamBreakTimer = new Timer();
        mBeamBreakTimer.reset();
        new Trigger(this::beamBreakIsTriggered).onTrue(Commands.runOnce(() -> mBeamBreakTimer.start()));
        new Trigger(() -> mBeamBreakTimer.hasElapsed(0.15)).onTrue(Commands.runOnce(() -> {
            if (mCurrentState == IntakeState.IN) {
                setState(IntakeState.OFF);
            }
            mBeamBreakTimer.stop();
            mBeamBreakTimer.reset();
        }));

        manualSetPoint = 0;

        mBeamBreakTopic = NetworkTableInstance.getDefault().getBooleanTopic("intake beam");
        mBeamBreakPublisher = mBeamBreakTopic.publish();
    }

    private CANSparkMax constructMaxMotor(MotorConstants motorValues){
        CANSparkMax motor = new CANSparkMax(motorValues.motorID(), motorValues.motorType());
        motor.restoreFactoryDefaults();
        motor.setInverted(motorValues.motorIsInverted());
        motor.setIdleMode(motorValues.idleMode());
        motor.setSmartCurrentLimit(motorValues.currentLimit());
        return motor;
    }
    private CANSparkFlex constructFlexMotor(MotorConstants motorValues){
        CANSparkFlex motor = new CANSparkFlex(motorValues.motorID(), motorValues.motorType());
        motor.restoreFactoryDefaults();
        motor.setInverted(motorValues.motorIsInverted());
        motor.setIdleMode(motorValues.idleMode());
        motor.setSmartCurrentLimit(motorValues.currentLimit());
        return motor;
    }

    private SparkPIDController constructPIDController(CANSparkBase motor, PIDConstants kPIDValues) {
        SparkPIDController pid = motor.getPIDController();

        pid.setP(kPIDValues.p());
        pid.setI(kPIDValues.i());
        pid.setD(kPIDValues.d());

        return pid;
    }

    public boolean beamBreakIsTriggered() {
        return !mBeamBreak.get();
    }

    public boolean beamBreakIsNotTriggered() {
        return !beamBreakIsTriggered();
    }


    @Deprecated
    public boolean getBeamBreakStatus() {
        return mBeamBreak.get();
    }

    /**
     * Gets the current instance of the intake subsystem.
     * 
     * @return The current instance of the intake subsystem
     */
    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    /**
     * Gets the current state of the intake.
     * 
     * @return The current state of the intake.
     */
    public IntakeState getState() {
        return mCurrentState;
    }

    /**
     * Sets the state of the intake.
     * 
     * @param state The desired state of the intake.
     */
    public void setState(IntakeState state) {
        mCurrentState = state;
        System.out.println("Setting to " + state);
    }

    /**
     * Returns a {@link Command} that sets the state of the intake.
     * 
     * @param state The desired state of the intake
     * @return A command that sets the state of the intake
     */
    public Command setStateCommand(IntakeState state) {
        return Commands.runOnce(() -> {
            setState(state);
        });
    }

    public void setPctgSpeed(double pctg) throws IllegalArgumentException{

        if(Math.abs(pctg)>1) {

            throw new IllegalArgumentException("pctg can't be bigger than 1");
        }
        manualSetPoint = pctg;
        setState(IntakeState.MANUAL);
        // mPIDController.setReference(pctg, ControlType.kDutyCycle);
        mMotor.set(pctg);
        mFollowerMotor.set(pctg * kMainToFollowRatio);
    }   

    private void in() {
        // if(kUseBeamBreak) {
        //     if (!mBeamBreak.get()) {
        //         System.out.println("beam break triggered");
        //         mCurrentState = IntakeState.OFF;
        //         off();
        //         return;
        //     }
        // }

        // double velocity = mEncoder.getVelocity();
        // double outputPower = computeOneSidedPControlOutput(velocity);
        // manualSetPoint = outputPower;
        // System.out.println(outputPower);
        // mPIDController.setReference(outputPower, ControlType.kDutyCycle);
        // mFollowPIDController.setReference(outputPower * kMainToFollowRatio, ControlType.kDutyCycle);
        manualSetPoint = 0.3;

        mMotor.set(0.4);
        mFollowerMotor.set(-0.3);

    }

    private void load() {
        //mPIDController.setReference(kLoadSpeed, ControlType.kDutyCycle);
        // mFollowPIDController.setReference(kLoadSpeed * kMainToFollowRatio, ControlType.kDutyCycle);
        mMotor.set(0.7);
        mFollowerMotor.set(-0.6);
    }

    private void out() {
        // mPIDController.setReference(kOutSpeed, ControlType.kDutyCycle);
        // mFollowPIDController.setReference(kOutSpeed * kMainToFollowRatio, ControlType.kDutyCycle);
        mMotor.set(-0.4);
        mFollowerMotor.set(0.3);
    }

    private void off() {
        // mPIDController.setReference(kOffSpeed, ControlType.kDutyCycle);
        // mFollowPIDController.setReference(kOffSpeed * kMainToFollowRatio, ControlType.kDutyCycle);
        mMotor.set(0);
        mFollowerMotor.set(0);
        manualSetPoint = 0;
    }

    @Override
    public void updateShuffleboard() {
        mIntakeStateWidget.setString(mCurrentState.name());
        mManualSetPointWidget.setDouble(manualSetPoint);
        mIntakeVelocity.setDouble(mEncoder.getVelocity());
        mIntakeMotorCurrent.setDouble(mMotor.getOutputCurrent());
        SmartDashboard.putBoolean("beam break", beamBreakIsTriggered());
        SmartDashboard.putNumber("flex enc", mFollowerMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("flex applied output", mFollowerMotor.getAppliedOutput());

        mBeamBreakPublisher.accept(mBeamBreak.get());
    }

    @Override
    public void periodic() {
        switch (mCurrentState) {
            case IN: //will intake until beamBreak has been triggered,
                in();
                break;
            case LOAD: //loads into shooter
                load();
                break;
            case OUT:
                out();
                break;
            case OFF:
                off();
                break;
            case MANUAL:
            // Speed should already have been set, so don't do anything
                break;
            case NONE:
                mPIDController.setReference(0, ControlType.kDutyCycle);
                mFollowPIDController.setReference(0, ControlType.kDutyCycle);
                
            break;
        }
        updateShuffleboard();
    }

    @Override
    public void modeSwitchAction() {
        mCurrentState = IntakeState.OFF;
    }

    // One sided P Control will only add extra power if the intake is going to slow.
    // It does not adjust if it is going too fast.  The idea is that we can be more 
    // aggressive on adding power if the note is stuck in the intake.

    public static double computeOneSidedPControlOutput(double velocity) {

        final double targetVel = 120; // RPM Need to tune
        final double baseFF = targetVel / (4500/4) * 1.1; //4 is the reduction
        final double maxPower = baseFF * 3;
        final double maxBaseDelta = maxPower - baseFF;
        // If the intake goes to 0, kick up to maxPower.  Needs to be tuned.
        final double P = maxBaseDelta / targetVel; 
        double error = targetVel - velocity;

        double addedPower = 0;
        if(error < 0) {
            error = 0;
        } else {
            addedPower = error * P;
        }

        // For testing: 
        // addedPower = 0;

        final double finalOutput = baseFF + addedPower;

        return finalOutput;
    }    
}
