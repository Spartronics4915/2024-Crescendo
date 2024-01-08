package com.spartronics4915.frc2024.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.spartronics4915.frc2024.util.Loggable;

import static com.spartronics4915.frc2024.Constants.Intake.kCurrentLimit;
import static com.spartronics4915.frc2024.Constants.Intake.kIdleMode;
import static com.spartronics4915.frc2024.Constants.Intake.kInSpeed;
import static com.spartronics4915.frc2024.Constants.Intake.kMotorID;
import static com.spartronics4915.frc2024.Constants.Intake.kMotorIsInverted;
import static com.spartronics4915.frc2024.Constants.Intake.kOffSpeed;
import static com.spartronics4915.frc2024.Constants.Intake.kOutSpeed;

public class Intake extends SubsystemBase implements Loggable {
    public static enum IntakeState {
        IN, OUT, OFF, NONE; // NONE is only here as the Shuffleboard default value for troubleshooting
    }

    private static Intake mInstance;

    private IntakeState mCurrentState;

    private ShuffleboardLayout mIntakeOverview;
    private SimpleWidget mIntakeStateWidget;

    private final CANSparkMax mMotor;

    private Intake() {
        mCurrentState = IntakeState.OFF;

        mIntakeOverview = Shuffleboard
                .getTab("Overview")
                .getLayout("Intake", BuiltInLayouts.kList)
                .withSize(2, 2);

        mIntakeStateWidget = mIntakeOverview
                .add("State", IntakeState.NONE)
                .withSize(2, 2);

        mMotor = new CANSparkMax(kMotorID, MotorType.kBrushless);

        mMotor.restoreFactoryDefaults();
        mMotor.setInverted(kMotorIsInverted);
        mMotor.setIdleMode(kIdleMode);
        mMotor.setSmartCurrentLimit(kCurrentLimit);
        mMotor.burnFlash();
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
        mMotor.set(kInSpeed);
    }

    private void out() {
        mMotor.set(kOutSpeed);
    }

    private void off() {
        mMotor.set(kOffSpeed);
    }

    @Override
    public void updateShuffleboard() {
        mIntakeStateWidget.getEntry().setValue(mCurrentState);
    }

    @Override
    public void periodic() {
        switch (mCurrentState) {
            case IN:
                in();
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
