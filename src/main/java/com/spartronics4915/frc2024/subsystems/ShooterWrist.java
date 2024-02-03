package com.spartronics4915.frc2024.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.spartronics4915.frc2024.Constants.ShooterWristConstants;
import com.spartronics4915.frc2024.Constants.GeneralConstants;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeWristConstants;
import com.spartronics4915.frc2024.Constants.ShooterWristConstants.ShooterWristState;
import com.spartronics4915.frc2024.ShuffleBoard.ShooterWristTabManager;
import com.spartronics4915.frc2024.ShuffleBoard.ShooterWristTabManager.ShooterWristSubsystemEntries;
import com.spartronics4915.frc2024.Robot;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.SimType;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.SimulatorSettings;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.TrapezoidSimulatorInterface;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;
import com.spartronics4915.frc2024.util.MotorConstants;
import com.spartronics4915.frc2024.util.PIDConstants;
import com.spartronics4915.frc2024.util.TrapezoidSubsystemInterface;

import edu.wpi.first.math.controller.ArmFeedforward;
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
import static com.spartronics4915.frc2024.Constants.ShooterWristConstants.*; 
import java.util.function.*;


public class ShooterWrist extends SubsystemBase implements TrapezoidSimulatorInterface, TrapezoidSubsystemInterface {
    
    //#region variables

    private CANSparkMax mWristMotor;
    private SparkPIDController mPidController;

    private RelativeEncoder mEncoder;
    private TrapezoidProfile kTrapezoidProfile;

    private State mCurrentState;
    private Rotation2d mTargetRotation2d;
    private Rotation2d mManualDelta;
    private final ArmFeedforward kFeedforwardCalc;

    private static ShooterWrist mInstance;

    private boolean mManualMovement = false; //used to pause position setting to avoid conflict (if using Trapezoid movment due to the constant calls)

    private GenericEntry mShooterSetPointEntry;
    private GenericEntry mShooterEncoderReadingEntry;
    private GenericEntry mShooterManualControlEntry;

    //#endregion

    public static ShooterWrist getInstance() {
        if (mInstance == null) {
            mInstance = new ShooterWrist();
        }
        return mInstance;
    }

    public ShooterWrist() {
        super();
        mWristMotor = initMotor(ShooterWristConstants.kMotorConstants);
        mPidController = initPID(ShooterWristConstants.kPIDconstants);
        mEncoder = initEncoder();
        kTrapezoidProfile = initTrapezoid(ShooterWristConstants.kTrapzoidConstants);
        
        kFeedforwardCalc = initFeedForward();

        currentToSetPoint();

        initShuffleBoard();
    }

    //#region init functions

    private void initShuffleBoard() {
        var mEntries = ShooterWristTabManager.getEnumMap(this);
        mShooterSetPointEntry = mEntries.get(ShooterWristSubsystemEntries.ShooterSetPoint);
        mShooterEncoderReadingEntry = mEntries.get(ShooterWristSubsystemEntries.ShooterEncoderReading);
        mShooterManualControlEntry = mEntries.get(ShooterWristSubsystemEntries.ShooterManualControl);
    }

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

    private RelativeEncoder initEncoder(){ //TODO encoder init settings
        return mWristMotor.getEncoder();
    }

    private TrapezoidProfile initTrapezoid(Constraints constraints) {
        return new TrapezoidProfile(new Constraints(constraints.maxVelocity, constraints.maxAcceleration));
    }


    private ArmFeedforward initFeedForward(){
        var out = new ArmFeedforward(kWristFeedForward.kS(), kWristFeedForward.kG(), kWristFeedForward.kV(), kWristFeedForward.kA());
        
        return out;
    }

    //#endregion
    
    //#region component fucntions

    private Rotation2d getEncoderPosReading(){
        return Rotation2d.fromRotations(mEncoder.getPosition()); //CHECKUP Failure Point?
    }

    private double getEncoderVelReading(){
        return mEncoder.getVelocity(); //CHECKUP Failure Point?
    }

    private void setManualDelta(Rotation2d deltaPosition){
        mManualMovement = true;
        mManualDelta = deltaPosition;
    }

    private void setRotationSetPoint(Rotation2d angle){
        mTargetRotation2d = angle;
    }

    private void setState(ShooterWristState newState){
        mManualMovement = false;
        setRotationSetPoint(newState.shooterAngle);
    }

    private void currentToSetPoint(){
        mCurrentState = new State(getEncoderPosReading().getRotations(), 0);
        setRotationSetPoint(getEncoderPosReading()); //TODO clamp for saftey? for now will have force boolean
    }

    //#endregion

    //#region Commands

    public Command angleToSupplierCommand(Supplier<Rotation2d> supplier){
        return Commands.run(() -> {
            setRotationSetPoint(supplier.get());
        }, this);
    }

    public Command setStateCommand(ShooterWristState newState){
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
        if (mManualMovement) {
            manualControlUpdate();
        }
        TrapezoidMotionProfileUpdate();

        updateShuffle();
        //will add things here if trapezoid motion profiles get used
    }
    
    private void updateShuffle() {
        mShooterSetPointEntry.setDouble(mTargetRotation2d.getDegrees());
        mShooterEncoderReadingEntry.setDouble(getEncoderPosReading().getDegrees());
        mShooterManualControlEntry.setBoolean(mManualMovement);
    }

    private double getFeedForwardValue(){

        return kFeedforwardCalc.calculate(
            getEncoderPosReading().getRadians(), 
            (getEncoderVelReading() / 60.0) * 2 * Math.PI //convert from RPM --> Rads/s
        );
    }
    
    private void manualControlUpdate(){ //HACK untested

        if (mTargetRotation2d.getRotations() % 1.0 - kMaxAngle.getRotations() > 0 && mManualDelta.getRotations() > 0) { //CHECKUP might not work
            mManualDelta = Rotation2d.fromRotations(0);
        } else if (mTargetRotation2d.getRotations() % 1.0 - kMinAngle.getRotations() < 0 && mManualDelta.getRotations() < 0) {
            mManualDelta =  Rotation2d.fromRotations(0);
        }
        mTargetRotation2d = Rotation2d.fromRadians(mTargetRotation2d.getRadians() + mManualDelta.getRadians());
    }

    private void TrapezoidMotionProfileUpdate(){
        //CHECKUP not sure if this will work
        mCurrentState = kTrapezoidProfile.calculate(
            GeneralConstants.kUpdateTime,
            mCurrentState,
            new State(mTargetRotation2d.getRotations(), 0)
        );
        
        mPidController.setReference(mCurrentState.position, ControlType.kPosition, 0, getFeedForwardValue()); //CHECKUP FF output? currently set to volatgage out instead of precentage out
    }

    //#endregion
    @Override
    public void setPositionToReal() {
        currentToSetPoint();
    }

    @Override
    public State getSetPoint() {
        return mCurrentState;
    }

    @Override
    public SimulatorSettings getSettings() {
        return new SimulatorSettings(
            "Shooter", 
            1.0, 
            0.0, 
            20.0,
            new Color8Bit(Color.kDarkGreen), 
            SimType.Angle, 
            new Translation2d(1.5, 0)
        );
    }

}
