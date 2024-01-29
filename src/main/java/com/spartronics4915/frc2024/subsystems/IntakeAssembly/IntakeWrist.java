package com.spartronics4915.frc2024.subsystems.IntakeAssembly;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.EnumMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeWristConstants;
import com.spartronics4915.frc2024.Constants.GeneralConstants;
import com.spartronics4915.frc2024.Constants.GeneralConstants.*;

import com.spartronics4915.frc2024.Constants.UtilRec.*;
import com.spartronics4915.frc2024.ShuffleBoard.IntakeTabManager;
import com.spartronics4915.frc2024.ShuffleBoard.IntakeWristTabManager;
import com.spartronics4915.frc2024.ShuffleBoard.IntakeTabManager.IntakeSubsystemEntries;
import com.spartronics4915.frc2024.ShuffleBoard.IntakeWristTabManager.WristSubsystemEntries;
import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.SimType;
import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.SimulatorSettings;
import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.TrapazoidSimulatorInterface;
import com.spartronics4915.frc2024.util.TrapazoidSubsystemInterface;

public class IntakeWrist extends SubsystemBase implements TrapazoidSubsystemInterface, TrapazoidSimulatorInterface{
    //Decision list:
    
    //#region variables

        private static IntakeWrist mInstance;

        private CANSparkMax mWristMotor;
        private SparkPIDController mPidController;
        private RelativeEncoder mEncoder;
        private Rotation2d mRotSetPoint;

        private State mCurrState = null;

        private final TrapezoidProfile kTrapazoidProfile;
        private boolean mManualMovment = false; //used to pause position setting to avoid conflict (if using trapazoid movment due to the constant calls)
        //limit switches?

        //#region ShuffleBoardEntries

        private GenericEntry mManualControlEntry;
        private GenericEntry mWristSetPoint;


        //#endregion
    //#endregion

    public IntakeWrist() {
        super();
        mWristMotor = initMotor(IntakeWristConstants.kMotorConstants);
        mPidController = initPID(IntakeWristConstants.kPIDconstants);
        mEncoder = initEncoder();
        kTrapazoidProfile = initTrapazoid(IntakeWristConstants.kTrapzoidConstants);
        
        
        mCurrState = new State(getEncoderPosReading().getRotations(), getEncoderVelReading());
        setState(IntakeWristConstants.kStartupState);

        shuffleInit();
    }

    public static IntakeWrist getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeWrist();
        }
        return mInstance;
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

        //CHECKUP Decide on Vel conversion Factor (aka use rpm?)
        //position Conversion not needed by using rotation2d

        return pid;
    }

    public RelativeEncoder initEncoder(){ //TODO encoder init settings
        return mWristMotor.getEncoder();
    }

    private void shuffleInit() {
        var mEntries = IntakeWristTabManager.getEnumMap(this);
        mManualControlEntry = mEntries.get(WristSubsystemEntries.WristManualControl);
        mWristSetPoint = mEntries.get(WristSubsystemEntries.WristSetPoint);

        System.out.println(mManualControlEntry);
        System.out.println(mWristSetPoint);
    }

    private TrapezoidProfile initTrapazoid(TrapazoidConstaintsConstants constraints) {
        return new TrapezoidProfile(new Constraints(constraints.kMaxVel(), constraints.kMaxAccel()));
    }

    //#endregion

    public Rotation2d getEncoderPosReading(){
        return Rotation2d.fromRotations(mEncoder.getPosition()); //CHECKUP Failure Point?
    }

    public double getEncoderVelReading(){
        return mEncoder.getVelocity(); //CHECKUP Failure Point?
    }

    private boolean isSafeAngle(Rotation2d angle){
        return true; //TODO implement Safety
    }

    public void currentToSetPoint(){
        mCurrState = new State(getEncoderPosReading().getRotations(), getEncoderVelReading());
        setRotationSetPoint(getEncoderPosReading(), true); //TODO clamp for saftey? for now will have force boolean
    }
    
    private void setRotationSetPoint(Rotation2d angle, boolean force){
        if (isSafeAngle(angle) || force) //TODO remove force for setting to closest safe value or shutdown (based on context)
            mRotSetPoint = angle;
    }

    private void setVelocitySetPoint(double velocity){ //TODO units determiend by vel conversion factor
        mManualMovment = true;
        mPidController.setReference(velocity, ControlType.kVelocity); //CHECKUP override?
    }

    private void setState(IntakeAssemblyState newState){
        mManualMovment = false;
        setRotationSetPoint(newState.wristAngle, false);
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

        updateShuffleboard();
    }

    private void updateShuffleboard() {
        mManualControlEntry.setBoolean(mManualMovment);
        mWristSetPoint.setDouble(mRotSetPoint.getDegrees());
    }


    private void TrapazoidMotionProfileUpdate(){
        //CHECKUP not sure if this will work
        //can throw feedforward here if needed

        mCurrState = kTrapazoidProfile.calculate(
            GeneralConstants.kUpdateTime,
            mCurrState,
            new State(mRotSetPoint.getRotations(), 0)
        );
        
        mPidController.setReference(mCurrState.position, ControlType.kPosition);
    }

    @Override
    public void setPositionToReal() {
        currentToSetPoint();
    }

    @Override
    public State getSetPoint() {
        return mCurrState;
    }

    @Override
    public SimulatorSettings getSettings() {
        return new SimulatorSettings(
            "Wrist", 
            1.0, 
            0.0, 
            20.0,
            new Color8Bit(Color.kBlueViolet), 
            SimType.Angle, 
            new Translation2d(0, 0)
        );
    }
}