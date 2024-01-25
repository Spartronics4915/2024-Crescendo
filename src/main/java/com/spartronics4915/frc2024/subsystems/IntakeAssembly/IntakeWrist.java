package com.spartronics4915.frc2024.subsystems.IntakeAssembly;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeWristConstants;

import com.spartronics4915.frc2024.Constants.UtilRec.*;

public class IntakeWrist extends SubsystemBase{
    
    //#region variables
        private CANSparkMax mWristMotor;
        private SparkPIDController mPidController;
        private RelativeEncoder mEncoder;
        //Setpoint
        //limit switches?
        private boolean mManualMovment = false; //used to pause position setting to avoid conflict (if using trapazoid movment due to the constant calls)
    //#endregion

    public IntakeWrist() {
        super();
        mWristMotor = initMotor(IntakeWristConstants.kMotorConstants);
        mPidController = initPID(IntakeWristConstants.kPIDconstants);
        mEncoder = initEncoder();
    }


    //#region Init functions

    public CANSparkMax initMotor(MotorContstants motorValues){
        CANSparkMax motor = new CANSparkMax(motorValues.kMotorID(), motorValues.kMotorType());
        motor.restoreFactoryDefaults();
        motor.setInverted(motorValues.kMotorIsInverted());
        motor.setIdleMode(motorValues.kIdleMode());
        motor.setSmartCurrentLimit(motorValues.kCurrentLimit());
        motor.burnFlash();
        return motor;
    }

    public SparkPIDController initPID(PIDConstants kPIDValues){
        SparkPIDController pid = mWristMotor.getPIDController();

        pid.setP(kPIDValues.P());
        pid.setI(kPIDValues.I());
        pid.setD(kPIDValues.D());

        return pid;
    }

    public RelativeEncoder initEncoder(){
        return mWristMotor.getEncoder();
    }

    //#endregion

    private Rotation2d getEncoderRead(){
        return Rotation2d.fromDegrees(0.0); //TODO
    }

    public void setRotationSetPoint(Rotation2d angle){
        //TODO
    }

    public void setState(IntakeAssemblyState newState){
        mManualMovment = false;
        setRotationSetPoint(newState.wristAngle);
    }


    //#region Commands
    public Command setStateCommand(IntakeAssemblyState newState){
        return Commands.runOnce(() -> {
            setState(newState);
        });
    }


    public Command currentToSetPoint(){
        return Commands.runOnce(() -> {
            setRotationSetPoint(getEncoderRead());
        });
    }
    //#endregion

    @Override
    public void periodic() {
        if (mManualMovment) {
            // TODO Auto-generated method stub
            super.periodic();
        }
    }
}
