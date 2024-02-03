package com.spartronics4915.frc2024.subsystems.IntakeAssembly;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeWristConstants;
import com.spartronics4915.frc2024.Robot;
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
    //0 = down, 90 = horizantal, 180 = straight up
    
    //#region variables

        private static IntakeWrist mInstance;

        private CANSparkMax mWristMotor;
        private SparkPIDController mWristPIDController;

        private RelativeEncoder mEncoder;
        private Rotation2d mRotSetPoint;
        private Rotation2d mManualDelta;

        private final ArmFeedforward kFeedforwardCalc;

        private State mCurrState = null;

        private final TrapezoidProfile kTrapezoidProfile;
        private boolean mManualMovement = false; //used to pause position setting to avoid conflict (if using trapezoid movment due to the constant calls)
        //limit switches?

        //#region ShuffleBoardEntries

        private GenericEntry mManualControlEntry;
        private GenericEntry mWristSetPoint;


        //#endregion
    //#endregion

    public IntakeWrist() {
        super();
        mWristMotor = initMotor(IntakeWristConstants.kMotorConstants);
        mWristPIDController = initPID(IntakeWristConstants.kPIDConstants);
        mEncoder = initEncoder();
        kTrapezoidProfile = initTrapezoid(IntakeWristConstants.kTrapzoidConstraints);
        kFeedforwardCalc = initFeedForward();
        
        mEncoder.setPosition(0.0);

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

    private SparkPIDController initPID(PIDConstants kPIDValues){
        SparkPIDController pid = mWristMotor.getPIDController();

        pid.setP(kPIDValues.p());
        pid.setI(kPIDValues.i());
        pid.setD(kPIDValues.d());

        //CHECKUP Decide on Vel conversion Factor (aka use rpm?)
        //position Conversion not needed by using rotation2d

        return pid;
    }

    private RelativeEncoder initEncoder(){ //Add Offset
        return mWristMotor.getEncoder();
    }

    private void shuffleInit() {
        var mEntries = IntakeWristTabManager.getEnumMap(this);
        mManualControlEntry = mEntries.get(WristSubsystemEntries.WristManualControl);
        mWristSetPoint = mEntries.get(WristSubsystemEntries.WristSetPoint);
    }

    private TrapezoidProfile initTrapezoid(Constraints constraints) {
        return new TrapezoidProfile(constraints);
    }

    private ArmFeedforward initFeedForward(){
        var out = new ArmFeedforward(kArmFeedForward.kS(), kArmFeedForward.kG(), kArmFeedForward.kV(), kArmFeedForward.kA());
        
        return out;
    }

    //#endregion

    //#region Component Functions
    
    private Rotation2d getEncoderPosReading(){ //90 = horizantal, 0 = veritcally down(based on FF calculator)
        return Rotation2d.fromRotations(mEncoder.getPosition()); //CHECKUP Failure Point?
    }

    private double getEncoderVelReading(){
        return mEncoder.getVelocity(); //CHECKUP Failure Point?
    }

    private void currentToSetPoint(){
        mCurrState = new State(getEncoderPosReading().getRotations(), 0.0);
        setRotationSetPoint(getEncoderPosReading());
    }
    
    private void setRotationSetPoint(Rotation2d angle){
        mRotSetPoint = angle;
    }

    private void setManualDelta(Rotation2d deltaPosition){
        mManualMovement = true;
        mManualDelta = deltaPosition;
    }

    private void setState(IntakeAssemblyState newState){
        mManualMovement = false;
        setRotationSetPoint(newState.wristAngle);
    }

    //#endregion

    //#region Commands
    public Command setStateCommand(IntakeAssemblyState newState){
        return Commands.runOnce(() -> {
            setState(newState);
        });
    }


    public Command manualRunCommand(Rotation2d angleDelta){
        return this.startEnd(
            () -> setManualDelta(angleDelta), 
            () -> {
                if (!Robot.isSimulation()) currentToSetPoint(); //CHECKUP remove sim statment when doing chassis stuff?
                mManualMovement = false;
            }
        );
    }


    //#endregion

    //#region periodic functions

    @Override
    public void periodic() {
        //TODO add system to talk to elevator 
        
        if (mManualMovement) {
            manualControlUpdate();
        }
        TrapezoidMotionProfileUpdate();
        //will add things here if trapezoid motion profiles get used
        updateShuffleboard();
    }
    
    public boolean needSoftLimit(){
        //TODO implement elevator distance check
        return (/*get elevator height */ 0.0 > kMeterSafteyLimit);
    }
    
    private double getFeedForwardValue(){

        return kFeedforwardCalc.calculate(
            getEncoderPosReading().getRadians(), 
            (getEncoderVelReading() / 60.0) * 2 * Math.PI //convert from RPM --> Rads/s
        );
    }
    
    private void updateShuffleboard() {
        mManualControlEntry.setBoolean(mManualMovement);
        mWristSetPoint.setDouble(mRotSetPoint.getDegrees());
    }

    private void manualControlUpdate(){ //HACK untested

        if (mRotSetPoint.getRotations() % 1.0 - kMaxAngleAmp.getRotations() > 0 && mManualDelta.getRotations() > 0 && needSoftLimit()) { //CHECKUP might not work
            mManualDelta = Rotation2d.fromRotations(0);
        } else if (mRotSetPoint.getRotations() % 1.0 - kMinAngle.getRotations() < 0 && mManualDelta.getRotations() < 0) {
            mManualDelta =  Rotation2d.fromRotations(0);
        }
        mRotSetPoint = Rotation2d.fromRadians(mRotSetPoint.getRadians() + mManualDelta.getRadians());
    }

    private void TrapezoidMotionProfileUpdate(){
        //CHECKUP not sure if this will work
        mCurrState = kTrapezoidProfile.calculate(
            GeneralConstants.kUpdateTime,
            mCurrState,
            new State(mRotSetPoint.getRotations(), 0)
        );
        
        mWristPIDController.setReference(mCurrState.position, ControlType.kPosition, 0, getFeedForwardValue()); //CHECKUP FF output? currently set to volatgage out instead of precentage out
    }

    //#endregion

    //#region overriden interface methods

    @Override
    public void setPositionToReal() {
        currentToSetPoint();
    }

    @Override
    public State getSetPoint() {
        return new State(mCurrState.position - 0.25, mCurrState.velocity);
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
            new Translation2d(0.20, 1.5)
        );
    }

    //#endregion
}
