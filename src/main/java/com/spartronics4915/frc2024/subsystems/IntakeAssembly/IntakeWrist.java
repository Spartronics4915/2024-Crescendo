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


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeWristConstants;
import com.spartronics4915.frc2024.Constants.GeneralConstants;

import com.spartronics4915.frc2024.ShuffleBoard.IntakeWristTabManager;
import com.spartronics4915.frc2024.ShuffleBoard.IntakeWristTabManager.WristSubsystemEntries;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.SimType;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.SimulatorSettings;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.TrapezoidSimulatorInterface;
import com.spartronics4915.frc2024.util.MotorConstants;
import com.spartronics4915.frc2024.util.PIDConstants;
import com.spartronics4915.frc2024.util.TrapezoidSubsystemInterface;

import static com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeWristConstants.*;

public class IntakeWrist extends SubsystemBase implements TrapezoidSubsystemInterface, TrapezoidSimulatorInterface{
    //Decision list:
    
    //#region variables

        private static IntakeWrist mInstance;

        private CANSparkMax mWristMotor;
        private SparkPIDController mPidPosController;

        private RelativeEncoder mEncoder;
        private Rotation2d mRotSetPoint;

        private State mCurrState = null;

        private final TrapezoidProfile kTrapezoidProfile;
        private boolean mManualMovment = false; //used to pause position setting to avoid conflict (if using trapezoid movment due to the constant calls)
        //limit switches?

        //#region ShuffleBoardEntries

        private GenericEntry mManualControlEntry;
        private GenericEntry mWristSetPoint;


        //#endregion
    //#endregion

    public IntakeWrist() {
        super();
        mWristMotor = initMotor(IntakeWristConstants.kMotorConstants);
        mPidPosController = initPID(IntakeWristConstants.kPIDSlotPosconstants, 0);
        initPID(IntakeWristConstants.kPIDSlotVeloconstants, 1);
        mEncoder = initEncoder();
        kTrapezoidProfile = initTrapezoid(IntakeWristConstants.kTrapzoidConstraints);
        
        
        currentToSetPoint();
        
        shuffleInit();
    }

    public static IntakeWrist getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeWrist();
        }
        return mInstance;
    }

    //#region Init functions

    private CANSparkMax initMotor(MotorConstants motorValues){
        CANSparkMax motor = new CANSparkMax(motorValues.motorID(), motorValues.motorType());
        motor.restoreFactoryDefaults();
        motor.setInverted(motorValues.motorIsInverted());
        motor.setIdleMode(motorValues.idleMode());
        motor.setSmartCurrentLimit(motorValues.currentLimit());
        motor.burnFlash();
        return motor;
    }

    private SparkPIDController initPID(PIDConstants kPIDValues, int slot){
        SparkPIDController pid = mWristMotor.getPIDController();

        pid.setP(kPIDValues.p(), slot);
        pid.setI(kPIDValues.i(), slot);
        pid.setD(kPIDValues.d(), slot);

        //CHECKUP Decide on Vel conversion Factor (aka use rpm?)
        //position Conversion not needed by using rotation2d

        return pid;
    }

    private RelativeEncoder initEncoder(){ //TODO encoder init settings
        return mWristMotor.getEncoder();
    }

    private void shuffleInit() {
        var mEntries = IntakeWristTabManager.getEnumMap(this);
        mManualControlEntry = mEntries.get(WristSubsystemEntries.WristManualControl);
        mWristSetPoint = mEntries.get(WristSubsystemEntries.WristSetPoint);

        System.out.println(mManualControlEntry);
        System.out.println(mWristSetPoint);
    }

    private TrapezoidProfile initTrapezoid(Constraints constraints) {
        return new TrapezoidProfile(constraints);
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

    private void currentToSetPoint(){
        mCurrState = new State(getEncoderPosReading().getRotations(), getEncoderVelReading());
        setRotationSetPoint(getEncoderPosReading(), true); //TODO clamp for saftey? for now will have force boolean
    }
    
    private void setRotationSetPoint(Rotation2d angle, boolean force){
        if (isSafeAngle(angle) || force) //TODO remove force for setting to closest safe value or shutdown (based on context)
            mRotSetPoint = angle;
    }

    private void setVelocitySetPoint(double velocity){ //TODO units determiend by vel conversion factor
        mManualMovment = true;
        mPidPosController.setReference(velocity, ControlType.kVelocity, kVelPIDSlot); //CHECKUP override?
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
            TrapezoidMotionProfileUpdate();
        }
        //will add things here if trapezoid motion profiles get used

        updateShuffleboard();
    }

    private void updateShuffleboard() {
        mManualControlEntry.setBoolean(mManualMovment);
        mWristSetPoint.setDouble(mRotSetPoint.getDegrees());
    }


    private void TrapezoidMotionProfileUpdate(){
        //CHECKUP not sure if this will work
        //can throw feedforward here if needed

        mCurrState = kTrapezoidProfile.calculate(
            GeneralConstants.kUpdateTime,
            mCurrState,
            new State(mRotSetPoint.getRotations(), 0)
        );
        
        mPidPosController.setReference(mCurrState.position, ControlType.kPosition, kPosPIDSlot);
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
            new Translation2d(1.5, 0)
        );
    }
}
