package com.spartronics4915.frc2024.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.spartronics4915.frc2024.Constants.UtilRec.MotorContstants;
import com.spartronics4915.frc2024.Constants.UtilRec.PIDConstants;
import com.spartronics4915.frc2024.ShuffleBoard.IntakeTabManager;
import com.spartronics4915.frc2024.ShuffleBoard.IntakeTabManager.IntakeSubsystemEntries;
import com.spartronics4915.frc2024.util.Loggable;


import static com. spartronics4915.frc2024.Constants.Intake.*;
import static com.spartronics4915.frc2024.Constants.OI.kIntakeBeamBreakID;

public class Intake extends SubsystemBase implements Loggable {
    public static enum IntakeState {
        IN, LOAD, OUT, OFF, NONE; // NONE is only here as the Shuffleboard default value for troubleshooting
    }

    private static Intake mInstance;

    private IntakeState mCurrentState;

    private GenericEntry mIntakeStateWidget;

    private final CANSparkMax mMotor;
    private final SparkMaxPIDController mPIDController;

    private final DigitalInput mBeamBreak;

    private Intake() {
        mCurrentState = IntakeState.OFF;

        var EntryMap = IntakeTabManager.getEnumMap(this);
        mIntakeStateWidget = EntryMap.get(IntakeSubsystemEntries.IntakeState);

        //motor setup 

        mMotor = constructMotor(kMotorConstants);

        //PID controller setup
        mPIDController = constructPIDController(mMotor, kPIDconstants);

        //beam break setup
        mBeamBreak = new DigitalInput(kIntakeBeamBreakID);
    }

    private CANSparkMax constructMotor(MotorContstants kMotorValues){
        CANSparkMax motor = new CANSparkMax(kMotorValues.kMotorID(), kMotorValues.kMotorType());
        motor.restoreFactoryDefaults();
        motor.setInverted(kMotorValues.kMotorIsInverted());
        motor.setIdleMode(kMotorValues.kIdleMode());
        motor.setSmartCurrentLimit(kMotorValues.kCurrentLimit());
        motor.burnFlash();
        return motor;
    }

    private SparkMaxPIDController constructPIDController(CANSparkMax motor, PIDConstants kPIDValues) {
        SparkMaxPIDController pid = motor.getPIDController();

        pid.setP(kPIDValues.P());
        pid.setI(kPIDValues.I());
        pid.setD(kPIDValues.D());

        return pid;
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
    }

    /**
     * Returns a {@link Command} that sets the state of the intake.
     * 
     * @param state The desired state of the intake
     * @return A command that sets the state of the intake
     */
    public Command setStateCommand(IntakeState state) {
        return runOnce(() -> {
            setState(state);
        });
    }

    private void in() {
        if (mBeamBreak.get()) {
            mCurrentState = IntakeState.OFF;
            off();
            return;
        }
        mPIDController.setReference(kInSpeed, ControlType.kVelocity);
    }

    private void load() {
        mPIDController.setReference(kLoadSpeed, ControlType.kVelocity);
    }

    private void out() {
        mPIDController.setReference(kOutSpeed, ControlType.kVelocity);
    }

    private void off() {
        mPIDController.setReference(kOffSpeed, ControlType.kVelocity);
    }

    @Override
    public void updateShuffleboard() {
        mIntakeStateWidget.setValue(mCurrentState);
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
            case NONE:
                mMotor.set(0);
                break;
        }
        updateShuffleboard();
    }
}
