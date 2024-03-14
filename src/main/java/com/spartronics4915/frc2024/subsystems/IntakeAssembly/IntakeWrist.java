package com.spartronics4915.frc2024.subsystems.IntakeAssembly;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.ElevatorConstants;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeWristConstants;
import com.spartronics4915.frc2024.Robot;
import com.spartronics4915.frc2024.Constants.GeneralConstants;

import com.spartronics4915.frc2024.ShuffleBoard.IntakeWristTabManager;
import com.spartronics4915.frc2024.ShuffleBoard.IntakeWristTabManager.WristSubsystemEntries;
import com.spartronics4915.frc2024.subsystems.Elevator;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.SimType;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.SimulatorSettings;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.TrapezoidSimulatorInterface;
import com.spartronics4915.frc2024.util.ModeSwitchInterface;
import com.spartronics4915.frc2024.util.MotorConstants;
import com.spartronics4915.frc2024.util.PIDConstants;

import static com.spartronics4915.frc2024.Constants.IntakeAssembly.ElevatorConstants.kMetersToRotation;
import static com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeWristConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

public class IntakeWrist extends SubsystemBase implements ModeSwitchInterface, TrapezoidSimulatorInterface{
    //TODO test CanCoder logic
    //0 = down, 90 = horizantal, 180 = straight up
    // RPM
    //#region variables

        private static IntakeWrist mInstance;

        private CANSparkMax mWristMotor;
        private SparkPIDController mWristPIDController;

        private RelativeEncoder mEncoder;
        private Rotation2d mRotSetPoint;
        private Rotation2d mManualDelta;

        // private CANcoder mCANCoder;

        private final ArmFeedforward kFeedforwardCalc;

        private State mCurrState = null;

        private final TrapezoidProfile kTrapezoidProfile;
        private boolean mManualMovement = false; //used to pause position setting to avoid conflict (if using trapezoid movment due to the constant calls)
        //limit switches?

        private Elevator mElevatorSubsystem = Elevator.getInstance();

        private DigitalInput mLimitSwitch;
        private boolean mHoming = false;

        private boolean startupHome = false;

        //#region ShuffleBoardEntries

        private GenericEntry mManualControlEntry;
        private GenericEntry mWristSetPointEntry;
        private GenericEntry mEncoderEntry;


        //#endregion
    //#endregion

    public IntakeWrist() {
        super();
        mWristMotor = initMotor(IntakeWristConstants.kMotorConstants);
        mWristPIDController = initPID(IntakeWristConstants.kPIDconstants);
        mEncoder = initEncoder();
        kTrapezoidProfile = initTrapezoid(IntakeWristConstants.kConstraints);
        kFeedforwardCalc = initFeedForward();
        
        // mCANCoder = new CANcoder(kCANCoderID);
        // mCANCoder
        //         .getConfigurator()
        //         .apply(new CANcoderConfiguration()
        //                 .withMagnetSensor(new MagnetSensorConfigs()
        //                         .withMagnetOffset(-Rotation2d.fromDegrees(kCANCoderOffset).getRotations())));

        // resetEncoder(getCanCoderAngle());
        resetEncoder(kStartingAngle);

        mLimitSwitch = initLimitSwitch();

        mWristMotor.burnFlash();



        shuffleInit();

        // homeMotor(Rotation2d.fromDegrees(1)); 
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
        return motor;
    }

    private SparkPIDController initPID(PIDConstants kPIDValues){
        SparkPIDController pid = mWristMotor.getPIDController();

        pid.setOutputRange(-0.2, 0.2);
        System.out.println(kPIDValues);
        pid.setP(kPIDValues.p());
        pid.setI(kPIDValues.i());
        pid.setD(kPIDValues.d());


        return pid;
    }

    private RelativeEncoder initEncoder(){ //Add Offset
        return mWristMotor.getEncoder();
    }

    private void shuffleInit() {
        var mEntries = IntakeWristTabManager.getEnumMap(this);
        mManualControlEntry = mEntries.get(WristSubsystemEntries.WristManualControl);
        mWristSetPointEntry = mEntries.get(WristSubsystemEntries.WristSetPoint);
        mEncoderEntry = mEntries.get(WristSubsystemEntries.WristEncoderReading);

        IntakeWristTabManager.addMotorControlWidget(this);
    }

    private TrapezoidProfile initTrapezoid(Constraints constraints) {
        return new TrapezoidProfile(constraints);
    }

    private ArmFeedforward initFeedForward(){
        var out = new ArmFeedforward(kArmFeedForward.kS(), kArmFeedForward.kG(), kArmFeedForward.kV(), kArmFeedForward.kA());
        
        return out;
    }

    private DigitalInput initLimitSwitch(){
        DigitalInput out = new DigitalInput(kLimitSwitchChannel);

        new Trigger(out::get).onTrue(new Command() {

            @Override
            public boolean runsWhenDisabled() {
                return false;
            }

            @Override
            public void execute() {
                resetEncoder(Rotation2d.fromRotations(kLimitSwitchEncoderReading));
                mManualMovement = false;
                mHoming = false;
                startupHome = true;
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        });

        return out;
    }

    //#endregion

    //#region Component Functions
    
    private Rotation2d getWristAngle(){ //90 = horizantal, 0 = vertically down (based on FF calculator)
        return Rotation2d.fromRotations(mEncoder.getPosition()).div(kWristToRotationsRate);
    }

    // private Rotation2d getCanCoderAngle(){
    //     return Rotation2d.fromRotations(mCANCoder.getAbsolutePosition().getValue());
    // }

    private double getEncoderVelReading(){
        return mEncoder.getVelocity(); 
    }

    private void resetEncoder(Rotation2d angle){
        mEncoder.setPosition(angle.getRotations() * kWristToRotationsRate);
        currentToSetPoint(angle);
    }

    private void currentToSetPoint(){
        currentToSetPoint(getWristAngle());
    }

    private void currentToSetPoint(Rotation2d setpoint){
        updateCurrStateToReal(setpoint);
        setRotationSetPoint(setpoint);
    }

    private void updateCurrStateToReal(Rotation2d real){
        mCurrState = new State(real.getRotations(), 0.0);
    }
    
    private void setRotationSetPoint(Rotation2d angle){
        mRotSetPoint = angle;
    }

    private void setManualDelta(Rotation2d deltaPosition){
        mHoming = false;
        mManualMovement = true;
        mManualDelta = deltaPosition;
    }

    private void setState(IntakeAssemblyState newState){
        mHoming = false;
        mManualMovement = false;
        setRotationSetPoint(newState.wristAngle);
    }

    private void homeMotor(Rotation2d deltaPosition){
        mHoming = true;
        setManualDelta(deltaPosition);
    }

    //#endregion

    //#region Commands
    public Command setStateCommand(IntakeAssemblyState newState){
        return Commands.runOnce(() -> {
            setState(newState);
        });
    }

    public Command setPidConstant(PIDConstants values){
        return runOnce(() -> {
            mWristPIDController.setP(values.p());
            mWristPIDController.setI(values.i());
            mWristPIDController.setD(values.d());
        });
    }

    public Command setRotationSetpointTesting(double degrees){
        return Commands.runOnce(() -> {
            System.out.println("setting to: " + degrees);
            setRotationSetPoint(Rotation2d.fromDegrees(degrees));
        });
    }

    public Command homingCommand(Rotation2d angleDelta){
        return this.runOnce(() -> {
            homeMotor(angleDelta);
        });
    }


    public Command manualRunCommand(Rotation2d angleDelta){
        return this.startEnd(
            () -> setManualDelta(angleDelta), 
            () -> {
                if (!Robot.isSimulation()) currentToSetPoint();
                mManualMovement = false;
            }
        );
    }

    public Command resetEncoderToAngle(double degrees){
        return Commands.runOnce(() -> {
            resetEncoder(Rotation2d.fromDegrees(degrees));
        });
    }
        
    public boolean atTargetState(double rotationThreshold){
        return (Math.abs(getWristAngle().getRotations() - mRotSetPoint.getRotations()) < rotationThreshold);
    }

    //#endregion

    //#region periodic functions
    // private DoubleArrayPublisher test = NetworkTableInstance.getDefault().getTable("lookHereForLogging").getDoubleArrayTopic("stuff").publish(); 

    @Override
    public void periodic() {        
        if (mManualMovement) {
            manualControlUpdate();
        }
        TrapezoidMotionProfileUpdate();
        //will add things here if trapezoid motion profiles get used
        updateShuffleboard();
        handleCANCoderLogic();

        // test.accept(new double[]{
        //     mWristMotor.getAppliedOutput(), //0
        //     mWristMotor.getOutputCurrent(), //1
        //     mEncoder.getPosition(), //2 
        //     getWristAngle().getDegrees(), //3 real position
        //     Units.rotationsToDegrees(mCurrState.position), //4 trapazoid position
        //     mRotSetPoint.getDegrees() - Units.rotationsToDegrees(mCurrState.position), //5 //trapazoid offset
        //     Units.rotationsToDegrees(mCurrState.position) - getWristAngle().getDegrees() //6 //PID P
        // });
    }
    
    public boolean needSoftLimit(){
        return (mElevatorSubsystem.getHeight()  > kMeterSafetyLimit + (ElevatorConstants.kMaxMeters-kMeterSafetyLimit)/2) || mElevatorSubsystem.getSetpointHeight() > kMeterSafetyLimit;
    }
    
    private double getFeedForwardValue(){

        // return kFeedforwardCalc.calculate(
        //     getWristAngle().getRadians(), 
        //     (getEncoderVelReading() / 60.0) * 2 * Math.PI //convert from RPM --> Rads/s
        // );
        return 0.0;
    }
    
    private void updateShuffleboard() {
        mManualControlEntry.setBoolean(mManualMovement);
        mWristSetPointEntry.setDouble(mRotSetPoint.getDegrees());
        mEncoderEntry.setDouble(getWristAngle().getDegrees());
    }

    private void manualControlUpdate(){
        mRotSetPoint = Rotation2d.fromRadians(mRotSetPoint.getRadians() + mManualDelta.getRadians());
    }

    private void TrapezoidMotionProfileUpdate(){
        //CHECKUP not sure if this will work

        if (!mHoming)
            mRotSetPoint = Rotation2d.fromRotations(
                MathUtil.clamp(mRotSetPoint.getRotations(), kMinAngle.getRotations(), (needSoftLimit()) ? kMaxAngleAmp.getRotations() : kMaxAngleGround.getRotations())
            );
        
        mCurrState = kTrapezoidProfile.calculate(
            GeneralConstants.kUpdateTime,
            mCurrState,
            new State(mRotSetPoint.getRotations(), 0)
        );
        mWristPIDController.setReference(mCurrState.position * kWristToRotationsRate, ControlType.kPosition, 0, 0.0);
    }

    private void handleCANCoderLogic(){
        // System.out.println(getCanCoderAngle());
        // if (getCanCoderAngle().getDegrees() - getWristAngle().getDegrees() < kCanCoderResetAngle.getDegrees()) {
        //     resetEncoder(getCanCoderAngle());
        // }
    }

    //#endregion

    //#region overriden interface methods

    @Override
    public void modeSwitchAction() {
        currentToSetPoint();
        mManualMovement = false;
    }

    @Override
    public State getSimulatedSetPoint() {
        return new State(mCurrState.position - 0.5, mCurrState.velocity);
    }

    @Override
    public SimulatorSettings getSettings() {
        return new SimulatorSettings(
            "Wrist", 
            0.5,
            0.0, 
            5.0,
            new Color8Bit(Color.kBlueViolet), 
            SimType.Angle, 
            new Translation2d(0.20, 1.5)
        );
    }

    //#endregion
}
