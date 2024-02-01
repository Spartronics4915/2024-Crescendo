package com.spartronics4915.frc2024.subsystems;

import static com.spartronics4915.frc2024.Constants.ShooterWristConstants.kMotorConstants;

import com.ctre.phoenix6.controls.Follower;

import static com.spartronics4915.frc2024.Constants.ShooterConstants.kOffSpeed;
import static com.spartronics4915.frc2024.Constants.ShooterConstants.kPIDconstants;
import static com.spartronics4915.frc2024.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.Intake;
import com.spartronics4915.frc2024.util.Loggable;
import com.spartronics4915.frc2024.util.MotorConstants;
import com.spartronics4915.frc2024.util.PIDConstants;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements Loggable {
    
    public static enum ShooterState {
        ON, OFF, NONE; // NONE is only here as the Shuffleboard default value for troubleshooting
    }

    private static Shooter mInstance;

    private ShooterState mCurrentState;

    private GenericEntry mShooterStateWidget;

    private final CANSparkMax mShooterMotor;
    private final CANSparkMax mShooterFollowMotor;
    private final CANSparkMax mConveyorMotor;
    private final SparkPIDController mPIDController;
  
    public Shooter() {
        mCurrentState = ShooterState.OFF;
        mShooterMotor = constructMotor(kShooterMotorConstants);
        mConveyorMotor = constructMotor(kConveyorMotorConstants);
        mShooterFollowMotor = constructMotor(kShooterFollowMotorConstants);
        mShooterFollowMotor.follow(mShooterMotor, true);
        mPIDController = constructPIDController(mShooterMotor, kPIDconstants);
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

    private SparkPIDController constructPIDController(CANSparkMax motor, PIDConstants kPIDValues) {
        SparkPIDController pid = motor.getPIDController();

        pid.setP(kPIDValues.p());
        pid.setI(kPIDValues.i());
        pid.setD(kPIDValues.d());

        return pid;
    }

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    } 
    
    public ShooterState getState() {
        return mCurrentState;
    }

    public void setState(ShooterState state){
        mCurrentState = state;
    }

    private void shooterOff() {
        mCurrentState = ShooterState.OFF;
        mPIDController.setReference(kOffSpeed, ControlType.kVelocity);
    }

    private void shooterOn() {
        mCurrentState = ShooterState.ON;
        mPIDController.setReference(kOffSpeed, ControlType.kVelocity);
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

    @Override
    public void updateShuffleboard() {
        mShooterStateWidget.setString(mCurrentState.name());
    }

    @Override
    public void periodic() {
        updateShuffleboard();
    }

}
