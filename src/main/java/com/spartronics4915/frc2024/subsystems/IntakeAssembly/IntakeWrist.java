package com.spartronics4915.frc2024.subsystems.IntakeAssembly;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeWristConstants;

import com.spartronics4915.frc2024.Constants.UtilRec.*;

public class IntakeWrist extends SubsystemBase{
    //Decision list:
    //Trapazoid motion profiles? (can implement after the fact, structuring code so it can be easily done)
    //fully statebased or setpoint based? (I am going to go for setpoint based since it allows for manual controls)
    //another option is to implement smartmotion / smartvelocity
    
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

        //TODO Vel conversion Factor (rpm to xyz)
        //position Conversion not needed by using rotation2d

        return pid;
    }

    public RelativeEncoder initEncoder(){ //TODO encoder INIT
        return mWristMotor.getEncoder();
    }

    //#endregion

    private Rotation2d getEncoderRead(){
        return Rotation2d.fromDegrees(mEncoder.getPosition()); //CHECKUP Failure Point?
    }

    private boolean isSafeAngle(Rotation2d angle){
        return true; //TODO implement Safety
    }

    public void currentToSetPoint(){
        setRotationSetPoint(getEncoderRead()); //TODO clamp for saftey?
    }
    
    private void setRotationSetPoint(Rotation2d angle){
        if (isSafeAngle(angle))
            mPidController.setReference(angle.getRotations(), ControlType.kPosition); //CHECKUP setPos feels easy
    }

    private void setVelocitySetPoint(double velocity){ //TODO units determiend by vel conversion factor
        mPidController.setReference(velocity, ControlType.kVelocity);
    }

    private void setState(IntakeAssemblyState newState){
        mManualMovment = false;
        setRotationSetPoint(newState.wristAngle);
    }



    //#region Commands
    public Command setStateCommand(IntakeAssemblyState newState){
        return Commands.runOnce(() -> {
            setState(newState);
        });
    }


    public Command manualRunCommand(double wristSpeed){
        return Commands.startEnd(
            () -> setVelocitySetPoint(wristSpeed), 
            () -> currentToSetPoint()
        );
    }


    //#endregion

    @Override
    public void periodic() {
        //will add things here if trapazoid motion profiles get used
    }
}
