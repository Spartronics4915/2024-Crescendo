package com.spartronics4915.frc2024.subsystems.IntakeAssembly;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeWristConstants;
import com.spartronics4915.frc2024.Constants.GeneralConstants;
import com.spartronics4915.frc2024.Constants.GeneralConstants.*;

import com.spartronics4915.frc2024.Constants.UtilRec.*;

public class IntakeWrist extends SubsystemBase{
    //Decision list:
    
    //#region variables
        private CANSparkMax mWristMotor;
        private SparkPIDController mPidController;
        private RelativeEncoder mEncoder;
        //Setpoint
        private Rotation2d mRotSetPoint;

        private final TrapezoidProfile kTrapazoidProfile;
        //limit switches?
        private boolean mManualMovment = false; //used to pause position setting to avoid conflict (if using trapazoid movment due to the constant calls)
    //#endregion

    public IntakeWrist() {
        super();
        mWristMotor = initMotor(IntakeWristConstants.kMotorConstants);
        mPidController = initPID(IntakeWristConstants.kPIDconstants);
        mEncoder = initEncoder();
        kTrapazoidProfile = initTrapazoid(IntakeWristConstants.kTrapzoidConstants);

        setState(IntakeWristConstants.kStartupState);
    }


    //#region Init functions

    private TrapezoidProfile initTrapazoid(TrapazoidConstaintsConstants constraints) {
        return new TrapezoidProfile(new Constraints(constraints.kMaxVel(), constraints.kMaxAccel()));
    }

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

    private Rotation2d getEncoderPosReading(){
        return Rotation2d.fromRotations(mEncoder.getPosition()); //CHECKUP Failure Point?
    }

    private double getEncoderVelReading(){
        return mEncoder.getVelocity(); //CHECKUP Failure Point?
    }

    private boolean isSafeAngle(Rotation2d angle){
        return true; //TODO implement Safety
    }

    public void currentToSetPoint(){
        setRotationSetPoint(getEncoderPosReading()); //TODO clamp for saftey?
    }
    
    private void setRotationSetPoint(Rotation2d angle){
        if (isSafeAngle(angle))
            mRotSetPoint = angle;
    }

    private void setVelocitySetPoint(double velocity){ //TODO units determiend by vel conversion factor
        mManualMovment = true;
        mPidController.setReference(velocity, ControlType.kVelocity); //CHECKUP override?
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
        if (mManualMovment) {
            //let commands handle it
        } else {
            TrapazoidMotionProfileUpdate();
        }
        //will add things here if trapazoid motion profiles get used
    }

    private void TrapazoidMotionProfileUpdate(){
        //CHECKUP not sure if this will work
        //can throw feedforward here if needed
        var nextState = kTrapazoidProfile.calculate(
            GeneralConstants.kUpdateTime,
            new State(getEncoderPosReading().getRotations(), getEncoderVelReading()),
            new State(mRotSetPoint.getRotations(), 0)
        );
        mPidController.setReference(nextState.velocity, ControlType.kVelocity);
    }
}
