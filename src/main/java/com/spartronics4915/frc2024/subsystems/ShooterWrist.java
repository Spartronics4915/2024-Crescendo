package com.spartronics4915.frc2024.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.spartronics4915.frc2024.Constants.ShooterWristConstants;
import com.spartronics4915.frc2024.Constants.Drive.TrapazoidConstaintsConstants;
import com.spartronics4915.frc2024.Constants.GeneralConstants;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeWristConstants;
import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.SimType;
import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.SimulatorSettings;
import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.TrapazoidSimulatorInterface;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;
import com.spartronics4915.frc2024.util.MotorConstants;
import com.spartronics4915.frc2024.util.PIDConstants;
import com.spartronics4915.frc2024.util.TrapazoidSubsystemInterface;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterWrist extends SubsystemBase implements TrapazoidSimulatorInterface, TrapazoidSubsystemInterface {
    
    private CANSparkMax mShooterWristMotor;

    private CANSparkMax mWristMotor;
    private SparkPIDController mPidController;
    private RelativeEncoder mEncoder;
    private TrapezoidProfile kTrapazoidProfile;

    private State mCurrentState;
    private Rotation2d mTargetRotation2d; //won't simulate because this is null (as of Monday 1/29)

    private static ShooterWrist mInstance;

    private boolean mManualMovment = false; //used to pause position setting to avoid conflict (if using trapazoid movment due to the constant calls)

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
        kTrapazoidProfile = initTrapazoid(ShooterWristConstants.kTrapzoidConstants);
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

    private TrapezoidProfile initTrapazoid(TrapazoidConstaintsConstants constraints) {
        return new TrapezoidProfile(new Constraints(constraints.kMaxVel(), constraints.kMaxAccel()));
    }

    private Rotation2d getEncoderPosReading(){
        return Rotation2d.fromRotations(mEncoder.getPosition()); //CHECKUP Failure Point?
    }

    private double getEncoderVelReading(){
        return mEncoder.getVelocity(); //CHECKUP Failure Point?
    }

    private boolean isSafeAngle(Rotation2d angle){
        return true; //TODO implement Safety
    }

    private void setRotationSetPoint(Rotation2d angle, boolean force){
        if (isSafeAngle(angle) || force) //TODO remove force for setting to closest safe value or shutdown (based on context)
            mTargetRotation2d = angle;
    }

    private void currentToSetPoint(){
        mCurrentState = new State(getEncoderPosReading().getRotations(), getEncoderVelReading());
        setRotationSetPoint(getEncoderPosReading(), true); //TODO clamp for saftey? for now will have force boolean
    }

    @Override
    public void periodic() {
        super.periodic();
        if (mManualMovment) {
            //let commands handle it
        } else {
            TrapazoidMotionProfileUpdate();
        }
        
    }

    private void TrapazoidMotionProfileUpdate(){
        //CHECKUP not sure if this will work
        //can throw feedforward here if needed

        mCurrentState = kTrapazoidProfile.calculate(
            GeneralConstants.kUpdateTime,
            mCurrentState,
            new State(mTargetRotation2d.getRotations(), 0)
        );
        
        mPidController.setReference(mCurrentState.position, ControlType.kPosition);
    }

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
