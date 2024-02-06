package com.spartronics4915.frc2024.subsystems;

import static com.spartronics4915.frc2024.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.spartronics4915.frc2024.ShuffleBoard.ShooterTabManager;
import com.spartronics4915.frc2024.ShuffleBoard.ShooterTabManager.ShooterSubsystemEntries;
import com.spartronics4915.frc2024.util.Disableable;
import com.spartronics4915.frc2024.util.Loggable;
import com.spartronics4915.frc2024.util.ModeSwitchInterface;
import com.spartronics4915.frc2024.util.MotorConstants;
import com.spartronics4915.frc2024.util.PIDFConstants;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.spartronics4915.frc2024.RobotContainer.SubsystemFlags.*;
import java.util.Optional;


public class Shooter extends SubsystemBase implements Loggable, ModeSwitchInterface, Disableable {
    
    public static enum ShooterState {
        ON, OFF, NONE; // NONE is only here as the Shuffleboard default value for troubleshooting
    }

    public static enum ConveyorState {
        IN, OUT, OFF, NONE;
    }

    private static Shooter mInstance;

    private ShooterState mCurrentShooterState;
    private ConveyorState mCurrentConveyorState;

    private GenericEntry mShooterStateWidget;
    private GenericEntry mConveyerStateWidget;


    private final CANSparkMax mShooterMotor;
    private final CANSparkMax mShooterFollowMotor;
    private final CANSparkMax mConveyorMotor;
    private final SparkPIDController mPIDControllerLead; 
    private final SparkPIDController mPIDControllerFollow; 

    private final RelativeEncoder mShooterEncoder;


    private Shooter() {
        mCurrentShooterState = ShooterState.OFF;
        mCurrentConveyorState = ConveyorState.OFF;
        mShooterMotor = constructMotor(kShooterMotorConstants);
        mConveyorMotor = constructMotor(kConveyorMotorConstants);
        mShooterFollowMotor = constructMotor(kShooterFollowMotorConstants);
        // mShooterFollowMotor.follow(mShooterMotor, true);
        mPIDControllerLead = constructPIDController(mShooterMotor, kPIDconstants);
        mPIDControllerFollow = constructPIDController(mShooterFollowMotor, kPIDconstants);

        mShooterEncoder =  mShooterMotor.getEncoder();

        var mEntries = ShooterTabManager.getEnumMap(this);
        mShooterStateWidget = mEntries.get(ShooterSubsystemEntries.ShooterState);
        mConveyerStateWidget = mEntries.get(ShooterSubsystemEntries.ConveyorState);

    }

    @Override
    public void disable() {
        mConveyorMotor.disable();
        mShooterMotor.disable();
        mShooterFollowMotor.disable();
    }

    private CANSparkMax constructMotor(MotorConstants motorValues){
        CANSparkMax motor = new CANSparkMax(motorValues.motorID(), motorValues.motorType());
        motor.restoreFactoryDefaults();
        motor.setInverted(motorValues.motorIsInverted());
        motor.setIdleMode(motorValues.idleMode());
        motor.setSmartCurrentLimit(motorValues.currentLimit());
        motor.burnFlash();
        return motor;
    }

    private SparkPIDController constructPIDController(CANSparkMax motor, PIDFConstants kPIDValues) {
        SparkPIDController pid = motor.getPIDController();

        pid.setP(kPIDValues.p());
        pid.setI(kPIDValues.i());
        pid.setD(kPIDValues.d());
        pid.setFF(kPIDValues.ff());

        return pid;
    }

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    } 
    
    public ShooterState getShooterState() {
        return mCurrentShooterState;
    }

    public ConveyorState getConveyorState() {
        return mCurrentConveyorState;
    }

    public void setShooterState(ShooterState state){
        mCurrentShooterState = state;
    }

    public void setConveyorState(ConveyorState state){
        mCurrentConveyorState = state;
    }

    public Command setShooterStateCommand(ShooterState state) {
        return runOnce(() -> {
            setShooterState(state);
        });
    }

    public Command setConveyorStateCommand(ConveyorState state) {
        return runOnce(() -> {
            setConveyorState(state);
        });
    }

    private void shooterOff() {
        // mPIDControllerLead.setReference(kOffSpeed, ControlType.kVelocity);
        // mPIDControllerFollow.setReference(-kOffSpeed, ControlType.kVelocity);
        mShooterMotor.set(kOffSpeed);
        mShooterFollowMotor.set(kOffSpeed);

    }

    private void shooterOn() {
        // mPIDControllerLead.setReference(kShootSpeed, ControlType.kVelocity);
        // mPIDControllerFollow.setReference(-(kShootSpeed - kDiff), ControlType.kVelocity);

        mShooterMotor.set(kShootSpeed);
        mShooterFollowMotor.set(-(kShootSpeed - kDiff));
    }

    private void conveyorIn() {
        mConveyorMotor.set(kConveyorInSpeed);
    }

    private void conveyorOut() {
        mConveyorMotor.set(kConveyorOutSpeed);
    }

    private void conveyorOff() {
        mConveyorMotor.set(0);
    }

    public Optional<Boolean> hasSpunUp(){
        return  (ShooterFlag.isUsed) ? Optional.of(mShooterEncoder.getVelocity() >= kTargetRPM) : Optional.empty();
    }

    @Override
    public void updateShuffleboard() {
        mShooterStateWidget.setString(mCurrentShooterState.name());
        mConveyerStateWidget.setString(mCurrentConveyorState.name());
    }

    @Override
    public void periodic() {
        System.out.println("conveyer: " + mCurrentConveyorState + "\nShooter: " + mCurrentShooterState);

        switch (mCurrentShooterState) {
            case NONE:
            shooterOff();
                break;
            case OFF:
            shooterOff();
                break;
            case ON:
            shooterOn();
                break;
            
        }

        switch (mCurrentConveyorState) {
            case IN:
            conveyorIn();
                break;
            case NONE:
            conveyorOff();
                break;
            case OFF:
            conveyorOff();
                break;
            case OUT:
            conveyorOut();
                break;
            default:
                break;
            
        }
        updateShuffleboard();
    }

    @Override
    public void modeSwitchAction() {
        mCurrentShooterState = ShooterState.OFF;
        mCurrentConveyorState = ConveyorState.OFF;

    }

}
