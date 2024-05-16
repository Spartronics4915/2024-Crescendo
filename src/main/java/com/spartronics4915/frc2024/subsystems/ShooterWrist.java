package com.spartronics4915.frc2024.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.spartronics4915.frc2024.Constants.ShooterWristConstants;
import com.spartronics4915.frc2024.Constants.GeneralConstants;
import com.spartronics4915.frc2024.Constants.ShooterConstants;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeWristConstants;
import com.spartronics4915.frc2024.Constants.ShooterWristConstants.ShooterWristState;
import com.spartronics4915.frc2024.ShuffleBoard.ShooterWristTabManager;
import com.spartronics4915.frc2024.ShuffleBoard.ShooterWristTabManager.ShooterWristSubsystemEntries;
import com.spartronics4915.frc2024.Robot;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.SimType;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.SimulatorSettings;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.TrapezoidSimulatorInterface;
import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;
import com.spartronics4915.frc2024.subsystems.IntakeAssembly.IntakeWrist;
import com.spartronics4915.frc2024.util.ModeSwitchInterface;
import com.spartronics4915.frc2024.util.MotorConstants;
import com.spartronics4915.frc2024.util.PIDConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;

import java.util.Set;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.spartronics4915.frc2024.Constants.ShooterWristConstants.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.*;

import org.ejml.dense.row.decomposition.hessenberg.TridiagonalDecomposition_FDRB_to_FDRM;

public class ShooterWrist extends SubsystemBase implements TrapezoidSimulatorInterface, ModeSwitchInterface {
    //TODO tune PID values
    //TODO tune FF values

    // #region variables

    private CANSparkMax mWristMotor;
    private PIDController mPidController;

    private RelativeEncoder mEncoder;
    private TrapezoidProfile kTrapezoidProfile;

    private State mCurrentState;
    private Rotation2d mTargetRotation2d;
    private Rotation2d mManualDelta = new Rotation2d();
    
    private final ArmFeedforward kFeedforwardCalc;

    private static ShooterWrist mInstance;

    private boolean mManualMovement = false; // used to pause position setting to avoid conflict (if using Trapezoid
                                             // movment due to the constant calls)

    private DigitalInput mLimitSwitch;

    private GenericEntry mShooterSetPointEntry;
    private GenericEntry mShooterEncoderReadingEntry;
    private GenericEntry mShooterManualControlEntry;
    private GenericEntry mShooterDelta;
    private GenericEntry mAppliedOutput;
    private GenericEntry mShooterWristErrorPID;
    private GenericEntry mShooterWristErrorTrapazoid;
    private GenericEntry mShooterWristPigeonAngleReading;
    private GenericEntry mShooterWristPigeonDriftEntry;
    private GenericEntry mShooterWristPIDSetpoint;

    public static final double kOutputRange = 0.2;
    private static final double kRotationLockTolerance = -0.8;

    private LinearFilter mFilter;

    private SendableChooser<Rotation2d> chooser;

    private Rotation2d mPigeonDrift = new Rotation2d();

    private boolean startupHome = false;
    private boolean mHoming = false;

    private Pigeon2 mIMU;
    
    
    private final Pigeon2 mSwerveIMU = SwerveDrive.getInstance().getIMU();

    private StatusSignal<Double> mGravVectorX;
    private StatusSignal<Double> mGravVectorY;
    private StatusSignal<Double> mGravVectorZ;

    // #endregion

    public static ShooterWrist getInstance() {
        if (mInstance == null) {
            mInstance = new ShooterWrist();
        }
        return mInstance;
    }

    private ShooterWrist() {
        super();
        mIMU = new Pigeon2(24);

        mGravVectorX = mIMU.getGravityVectorX();
        mGravVectorY = mIMU.getGravityVectorY();
        mGravVectorZ = mIMU.getGravityVectorZ();

        BaseStatusSignal.setUpdateFrequencyForAll(50, mGravVectorX, mGravVectorY, mGravVectorZ);

        mWristMotor = initMotor(ShooterWristConstants.kMotorConstants);
        mPidController = initPID();
        mEncoder = initEncoder();
        kTrapezoidProfile = initTrapezoid();
        mLimitSwitch = initLimitSwitch();

        kFeedforwardCalc = initFeedForward();

        // resetEncoder(kStartingAngle);

        mFilter = LinearFilter.movingAverage(3);

        mWristMotor.burnFlash();


        currentToSetPoint();

        initShuffleBoard();
        Shuffleboard.getTab("ShooterWrist").addDouble("enc rot", mEncoder::getPosition);


        updateShooterPitchCache();
        resetEncoder(getCachedShooterPitch(), true);

        // new Trigger(() -> {
        //     return mIMUTimer.advanceIfElapsed(0.1);
        // }).onTrue(Commands.defer(() -> {
        //     return Commands.runOnce(() -> 
        //         mPigeonDrift = getShooterPitchFiltered().minus(getEncoderPos())
        //     );
        // }, Set.of()));


        new Trigger(mLimitSwitch::get).onTrue(new Command() {

            @Override
            public boolean runsWhenDisabled() {
                return false;
            }

            @Override
            public void execute() {
                setPosition(Rotation2d.fromRotations(kLimitSwitchEncoderReading));
                // if (mTargetRotation2d.getRotations() < kLimitSwitchEncoderReading * kInToOutRotations +
                // kLimitSwitchTriggerOffset) { //CHECKUP does trigger get hit rapidly
                currentToSetPoint(Rotation2d.fromRotations(kLimitSwitchEncoderReading));
                // }
                mHoming = false;
                mManualMovement = false;
                startupHome = true;
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        });

        // homeMotor(Rotation2d.fromDegrees(1)); 

    }


    // #region init functions

    private void initShuffleBoard() {
        var mEntries = ShooterWristTabManager.getEnumMap(this);
        ShooterWristTabManager.addMotorControlWidget(this);
        
        mShooterSetPointEntry = mEntries.get(ShooterWristSubsystemEntries.ShooterSetPoint);
        mShooterEncoderReadingEntry = mEntries.get(ShooterWristSubsystemEntries.ShooterEncoderReading);
        mShooterManualControlEntry = mEntries.get(ShooterWristSubsystemEntries.ShooterManualControl);
        mShooterDelta = mEntries.get(ShooterWristSubsystemEntries.ShooterDelta);
        mAppliedOutput = mEntries.get(ShooterWristSubsystemEntries.WristAppliedOutput);

        mShooterWristErrorPID = mEntries.get(ShooterWristSubsystemEntries.ShooterWristErrorPID);
        mShooterWristErrorTrapazoid = mEntries.get(ShooterWristSubsystemEntries.ShooterWristErrorTrapazoid);

        mShooterWristPigeonAngleReading = mEntries.get(ShooterWristSubsystemEntries.ShooterWristPigeonAngleReading);
        mShooterWristPigeonDriftEntry = mEntries.get(ShooterWristSubsystemEntries.ShooterWristPigeonDrift);

        mShooterWristPIDSetpoint = mEntries.get(ShooterWristSubsystemEntries.ShooterWristPIDSetpoint);

        chooser = new SendableChooser<Rotation2d>();

        chooser.addOption("MIN", kMinAngle);
        chooser.addOption(""+ShooterWristState.SUBWOOFER_SHOT.shooterAngle.getDegrees(), ShooterWristState.SUBWOOFER_SHOT.shooterAngle);
        chooser.addOption(""+ShooterWristState.STOW.shooterAngle.getDegrees(), ShooterWristState.STOW.shooterAngle);
        chooser.addOption("MAX", kMaxAngle);


        SmartDashboard.putData("choose shooterWrist value", chooser);
    }

    private CANSparkMax initMotor(MotorConstants motorValues) {
        CANSparkMax motor = new CANSparkMax(motorValues.motorID(), motorValues.motorType());
        motor.restoreFactoryDefaults();
        motor.setInverted(motorValues.motorIsInverted());
        motor.setIdleMode(motorValues.idleMode());
        motor.setSmartCurrentLimit(motorValues.currentLimit());
        return motor;
    }

    private PIDController initPID() {
        PIDController pid = new PIDController(
            kPIDconstants.p(), 
            kPIDconstants.i(), 
            kPIDconstants.d()
        );

        pid.setIZone(5.0 / 360.0);

        // SparkPIDController pid = mWristMotor.getPIDController();

        // pid.setP(kPIDconstants.p());
        // pid.setI(kPIDconstants.i());
        // pid.setD(kPIDconstants.d());

        // pid.setOutputRange(-kOutputRange, kOutputRange);


        // position Conversion not needed by using rotation2d

        return pid;
    }

    private RelativeEncoder initEncoder() {

        RelativeEncoder encoder = mWristMotor.getEncoder();
        // Set the encoder to zero for now to make sure it is initialized to a reasonable value.
        return encoder;
    }

    private TrapezoidProfile initTrapezoid() {
        return new TrapezoidProfile(kConstraints);
    }

    private ArmFeedforward initFeedForward() {
        var out = new ArmFeedforward(kWristFeedForward.kS(), kWristFeedForward.kG(), kWristFeedForward.kV(),
                kWristFeedForward.kA());

        return out;
    }

    private DigitalInput initLimitSwitch() {
        DigitalInput out = new DigitalInput(kLimitSwitchChannel);
        return out;
    }

    // #endregion

    // #region component fucntions
    private Rotation2d getEncoderPos(){
            return Rotation2d.fromRotations(mEncoder.getPosition()).div(kWristToRotationsRate); 
    }

    private double getEncoderVelReading() {
        return mEncoder.getVelocity() / kWristToRotationsRate;
    }

    private void setManualDelta(Rotation2d deltaPosition) {
        mHoming = false;
        mManualMovement = true;
        mManualDelta = deltaPosition;
    }

    private void homeMotor(Rotation2d deltaPosition) {
        mHoming = true;
        setManualDelta(deltaPosition);
    }

    private void setRotationSetPoint(Rotation2d angle) {
        mTargetRotation2d = angle;
    }

    public void publicSetRotationSetPoint(Rotation2d angle) {
        setRotationSetPoint(angle);
    }

    public Rotation2d getWristAngle() {
        return getCachedShooterPitch();
    }

    private void setPosition(Rotation2d newAngle){
        mEncoder.setPosition(newAngle.getRotations() * kWristToRotationsRate);
    }

    private void setState(ShooterWristState newState) {
        mManualMovement = false;
        publicSetRotationSetPoint(newState.shooterAngle);
    }

    private void currentToSetPoint() {
        currentToSetPoint(getWristAngle());
    }

    private void currentToSetPoint(Rotation2d setpoint) {
        currentToSetPoint(setpoint, true);
    }

    private void currentToSetPoint(Rotation2d setpoint, boolean resetTarget) {
        updateCurrStateToReal(setpoint);
        // System.out.println(setpoint);
        if(resetTarget) setRotationSetPoint(setpoint); 
    }

    private void updateCurrStateToReal(Rotation2d setpoint) {
        mCurrentState = new State(setpoint.getRotations(), 0.0);
    }

    public boolean atTarget() {
        return Math.abs(getWristAngle().getRotations() - mTargetRotation2d.getRotations()) < kAimedAtTargetThreshold;
    }


    
    public static Measure<Angle> findAngle(double x, double y){
        return Radians.of(Math.atan2(y, x));
    }

    /*
     * this determines if the shooterwrist should stop moving based on the gravity vector of the IMU
     * https://www.desmos.com/3d/43a3d2d01d (the area showing is where this function returns true)
     */
    private boolean getRotationLock(){
        return mSwerveIMU.getGravityVectorZ().getValueAsDouble() < 0.98;
    }


    // #endregion

    // #region Commands

    public Command resetEncoder() {
        return resetToAngle(0.0);
    }

    public Command resetToAngle(double degrees) {
        return runOnce(() -> {
            resetEncoder(Rotation2d.fromDegrees(degrees), true);
        });
    }

    public Command setPidConstant(PIDConstants values){
        return Commands.sequence(
            Commands.waitUntil(DriverStation::isEnabled).ignoringDisable(true),
            runOnce(() -> {
                System.out.println("setting pid values");
                mPidController.setP(values.p());
                mPidController.setI(values.i());
                mPidController.setD(values.d());
            })
        ).ignoringDisable(true);
    }

    public void resetEncoder(Rotation2d angle, boolean resetSetpoint){
        setPosition(angle);
        currentToSetPoint(angle, resetSetpoint);
    }

    public Command angleToSupplierCommand(Supplier<Rotation2d> supplier) {
        return Commands.run(() -> {
            setRotationSetPoint(supplier.get());
        }, this);
    }

    public Command setValueFromChooser(){
        return Commands.runOnce(() -> {
            setRotationSetPoint(chooser.getSelected());
        });
    }

    public Command setStateCommand(ShooterWristState newState) {
        return Commands.runOnce(() -> {
            setState(newState);
        });
    }

    public Command manualRunCommand(Rotation2d angleDelta) {
        return this.startEnd(
                () -> setManualDelta(angleDelta),
                () -> {
                    if (!Robot.isSimulation())
                        currentToSetPoint();
                    mManualMovement = false;
                });
    }

    public Command homeMotorCommand(Rotation2d angleDelta) {
        return runOnce(() -> {
            homeMotor(angleDelta);
        });
    }

    // #endregion

    // #region periodic functions

    private Rotation2d mPeriodicPigeonReading = new Rotation2d(); //this is to prevent a race condition during a single periodic cycle
    

    private Rotation2d getCachedShooterPitch(){
        return mPeriodicPigeonReading;
    }

    private void updateShooterPitchCache(){
        
        var xStream = mGravVectorX;
        var yStream = mGravVectorY;

        var d = findAngle(
            xStream.getValueAsDouble(), 
            yStream.getValueAsDouble()).in(Degrees);
        
        mPeriodicPigeonReading = Rotation2d.fromDegrees((d));
    }

    @Override
    public void periodic() {
        // mPigeonDrift = getShooterPitchFiltered().minus(getEncoderPos());
        //once per periodic update of angle
        updateShooterPitchCache();

        if (mManualMovement) {
            manualControlUpdate();
        }
        TrapezoidMotionProfileUpdate();

        updateShuffle();

        BaseStatusSignal.refreshAll(
            mGravVectorX,
            mGravVectorY,
            mGravVectorZ
        );

        // System.out.println(
        //     getShooterPitch().getDegrees() + "\t : " + 
        //     mIMU.getGravityVectorX().getValueAsDouble() + "\t : " +
        //     mIMU.getGravityVectorY().getValueAsDouble() + "\t : " + 
        //     mIMU.getGravityVectorZ().getValueAsDouble()
        // );
        // will add things here if trapezoid motion profiles get used
    }

    private void updateShuffle() {
        mShooterSetPointEntry.setDouble(mTargetRotation2d.getDegrees());
        mShooterEncoderReadingEntry.setDouble(getEncoderPos().getDegrees());
        mShooterManualControlEntry.setBoolean(mManualMovement);
        mShooterDelta.setDouble(mManualDelta.getDegrees());
        mAppliedOutput.setDouble(mWristMotor.getAppliedOutput());
        mShooterWristPigeonAngleReading.setDouble(getCachedShooterPitch().getDegrees());
        mShooterWristPigeonDriftEntry.setDouble(mPigeonDrift.getDegrees());
        SmartDashboard.putBoolean("shooter ls", mLimitSwitch.get());
    }

    private double getFeedForwardValue() {
        return kFeedforwardCalc.calculate(
                getWristAngle().getRadians(),
                ((getEncoderVelReading() / 60.0) * 2 * Math.PI) / kWristToRotationsRate // convert from RPM --> Rads/s
        );
        // return 0.0;
    }

    private void manualControlUpdate() {
        mTargetRotation2d = mTargetRotation2d.plus(mManualDelta);
    }

    private void TrapezoidMotionProfileUpdate() {
        if (!mHoming)
            mTargetRotation2d = Rotation2d.fromRotations(
                MathUtil.clamp(mTargetRotation2d.getRotations(), kMinAngle.getRotations(), kMaxAngle.getRotations())
            );

        mCurrentState = kTrapezoidProfile.calculate(
                GeneralConstants.kUpdateTime,
                mCurrentState,
                new State(mTargetRotation2d.getRotations(), 0));

        mShooterWristPIDSetpoint.setDouble(Rotation2d.fromRotations(mCurrentState.position).getDegrees());
        mShooterWristErrorPID.setDouble(Rotation2d.fromRotations(mCurrentState.position).getDegrees() - getWristAngle().getDegrees());
        mShooterWristErrorTrapazoid.setDouble(mTargetRotation2d.getDegrees() - Rotation2d.fromRotations(mCurrentState.position).getDegrees());

        if (!getRotationLock()) {
            // System.out.println(getRotationLock());
            mWristMotor.set(
                MathUtil.clamp(
                    mPidController.calculate(
                        getWristAngle().getRotations(), 
                        mCurrentState.position),
                        -0.05
                    -kOutputRange, kOutputRange)
            );
        } else{
            mWristMotor.set(0.0);
        }
        // mPidController.setReference((mCurrentState.position) * kWristToRotationsRate, ControlType.kPosition, 0, 0);
    }

    // #endregion
    @Override
    public void modeSwitchAction() {
        updateShooterPitchCache();
        currentToSetPoint();
        mManualMovement = false;
    }

    @Override
    public State getSimulatedSetPoint() {
        return new State(0.5 - mCurrentState.position, 0.0);
    }

    @Override
    public SimulatorSettings getSettings() {
        return new SimulatorSettings(
                "Shooter",
                1.0,
                0.0,
                5.0,
                new Color8Bit(Color.kDarkGreen),
                SimType.Angle,
                new Translation2d(2, 0));
    }

}
