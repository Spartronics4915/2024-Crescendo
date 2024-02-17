package com.spartronics4915.frc2024.subsystems.IntakeAssembly;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.spartronics4915.frc2024.util.MotorConstants;
import com.spartronics4915.frc2024.util.PIDConstants;
import com.spartronics4915.frc2024.ShuffleBoard.IntakeTabManager;
import com.spartronics4915.frc2024.ShuffleBoard.IntakeTabManager.IntakeSubsystemEntries;
import com.spartronics4915.frc2024.util.Loggable;
import com.spartronics4915.frc2024.util.ModeSwitchInterface;

import static com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeConstants.*;

public class Intake extends SubsystemBase implements Loggable, ModeSwitchInterface {
    public static enum IntakeState {
        IN, LOAD, OUT, OFF, NONE; // NONE is only here as the Shuffleboard default value for troubleshooting
    }

    private static Intake mInstance;

    private IntakeState mCurrentState;

    private GenericEntry mIntakeStateWidget;

    private final CANSparkMax mMotor;
    private final SparkPIDController mPIDController;

    private final DigitalInput mBeamBreak;

    private Intake() {
        mCurrentState = IntakeState.OFF;

        var EntryMap = IntakeTabManager.getEnumMap(this);
        mIntakeStateWidget = EntryMap.get(IntakeSubsystemEntries.IntakeState);

        //motor setup 

        mMotor = constructMotor(kMotorConstants);

        //PID controller setup
        mPIDController = constructPIDController(mMotor, kPIDconstants);

        mMotor.burnFlash();

        //beam break setup
        mBeamBreak = new DigitalInput(kIntakeBeamBreakID);
    }

    private CANSparkMax constructMotor(MotorConstants motorValues){
        CANSparkMax motor = new CANSparkMax(motorValues.motorID(), motorValues.motorType());
        motor.restoreFactoryDefaults();
        motor.setInverted(motorValues.motorIsInverted());
        motor.setIdleMode(motorValues.idleMode());
        motor.setSmartCurrentLimit(motorValues.currentLimit());
        return motor;
    }

    private SparkPIDController constructPIDController(CANSparkMax motor, PIDConstants kPIDValues) {
        SparkPIDController pid = motor.getPIDController();

        pid.setP(kPIDValues.p());
        pid.setI(kPIDValues.i());
        pid.setD(kPIDValues.d());

        return pid;
    }

    public boolean getBeamBreakStatus(){
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
        mIntakeStateWidget.setString(mCurrentState.name());
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

    @Override
    public void modeSwitchAction() {
        mCurrentState = IntakeState.OFF;
    }
}
