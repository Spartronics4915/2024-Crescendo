package com.spartronics4915.frc2024.subsystems;

import static com.spartronics4915.frc2024.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.spartronics4915.frc2024.RobotContainer;
import com.spartronics4915.frc2024.Constants.BlingModes;
import com.spartronics4915.frc2024.ShuffleBoard.ShooterTabManager;
import com.spartronics4915.frc2024.ShuffleBoard.ShooterTabManager.ShooterSubsystemEntries;
import com.spartronics4915.frc2024.util.Loggable;
import com.spartronics4915.frc2024.util.ModeSwitchInterface;
import com.spartronics4915.frc2024.util.MotorConstants;
import com.spartronics4915.frc2024.util.PIDConstants;
import com.spartronics4915.frc2024.util.PIDFConstants;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Shooter extends SubsystemBase implements Loggable, ModeSwitchInterface {
    public static final double ON_SPEED = 0.85;
    public static final double BACK_SPEED = 0.2;
    public static final double ROLLER_SPEED_DIFFERENCE = 0.1;

    public static enum ShooterState {
        ON, BACK, OFF, MANUAL, NONE; // NONE is only here as the Shuffleboard default value for troubleshooting
    }

    public static enum ConveyorState {
        IN, OUT, OFF, MANUAL, NONE, STORED, SHOOTING;
    }

    private static Shooter mInstance;

    private ShooterState mCurrentShooterState;
    private ConveyorState mCurrentConveyorState;

    private GenericEntry mShooterStateWidget;
    private GenericEntry mShooterSpeedWidget;
    private GenericEntry mShooterFollowSpeedWidget;
    private GenericEntry mConveyerStateWidget;

    private final CANSparkMax mShooterMotor;
    private final CANSparkMax mShooterFollowMotor;
    private final CANSparkMax mConveyorMotor;
    private final SparkPIDController mPIDControllerLead;
    private final SparkPIDController mPIDControllerFollow;

    private final RelativeEncoder mShooterEncoder;

    private final DigitalInput mBeamBreak;
    private final Timer mBeamBreakTimer;

    public Shooter() {
        mCurrentShooterState = ShooterState.OFF;
        mCurrentConveyorState = ConveyorState.OFF;
        mShooterMotor = constructMotor(kShooterMotorConstants);
        mShooterFollowMotor = constructMotor(kShooterFollowMotorConstants);
        mConveyorMotor = constructMotor(kConveyorMotorConstants);

        // mShooterFollowMotor.follow(mShooterMotor, true);
        mPIDControllerLead = constructPIDController(mShooterMotor, kPIDconstants);
        mPIDControllerFollow = constructPIDController(mShooterFollowMotor, kPIDconstants);

        mShooterMotor.burnFlash();
        mShooterFollowMotor.burnFlash();
        mConveyorMotor.burnFlash();

        mShooterEncoder = mShooterMotor.getEncoder();

        var mEntries = ShooterTabManager.getEnumMap(this);
        mShooterStateWidget = mEntries.get(ShooterSubsystemEntries.ShooterState);
        mConveyerStateWidget = mEntries.get(ShooterSubsystemEntries.ConveyorState);
        mShooterSpeedWidget = mEntries.get(ShooterSubsystemEntries.ShooterSpeed);
        mShooterFollowSpeedWidget = mEntries.get(ShooterSubsystemEntries.FollowShooterSpeed);

        ShooterTabManager.addMotorControlWidget(this);

        final var tab = Shuffleboard.getTab("Shooter");
        tab.addDouble("main applied", () -> mShooterMotor.getAppliedOutput());
        tab.addDouble("follower applied", () -> mShooterFollowMotor.getAppliedOutput());

        // Bling.addToLinkedList(new Bling.BlingMCwithPriority(() -> {
        // if (hasSpunUp()) {
        // return Optional.empty();
        // }
        // return Optional.of(new Bling.BlingMC(BlingModes.WARNING, Color.kOrange, Color.kOrangeRed));
        // }, 1));

        mBeamBreak = new DigitalInput(7);

        mBeamBreakTimer = new Timer();
        mBeamBreakTimer.reset();

        // If a note is stored in the shooter, we don't want this trigger running)
        new Trigger(this::beamBreakIsTriggered).onTrue(Commands.runOnce(mBeamBreakTimer::start));
        new Trigger(() -> mBeamBreakTimer.hasElapsed(0.15)).onTrue(Commands.runOnce(() -> {
            if (getConveyorState() != ConveyorState.STORED && (getConveyorState() != ConveyorState.SHOOTING)) {
                mBeamBreakTimer.stop();
                mBeamBreakTimer.reset();
                mCurrentConveyorState = ConveyorState.STORED;
                conveyorOff();
            }
        }));
    }

    public boolean beamBreakIsTriggered() {
        return !mBeamBreak.get(); // TODO: inverted or not?
    }

    public boolean beamBreakIsNotTriggered() {
        return !beamBreakIsTriggered();
    }

    private CANSparkMax constructMotor(MotorConstants motorValues) {
        CANSparkMax motor = new CANSparkMax(motorValues.motorID(), motorValues.motorType());
        motor.restoreFactoryDefaults();
        motor.setInverted(motorValues.motorIsInverted());
        motor.setIdleMode(motorValues.idleMode());
        motor.setSmartCurrentLimit(motorValues.currentLimit());
        return motor;
    }

    private SparkPIDController constructPIDController(CANSparkMax motor, PIDFConstants kpidconstants) {
        SparkPIDController pid = motor.getPIDController();

        pid.setP(kPIDconstants.p());
        pid.setI(kPIDconstants.i());
        pid.setD(kPIDconstants.d());
        pid.setFF(kPIDconstants.ff());

        return pid;
    }

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    public ShooterState getShooterState() {
        return mCurrentShooterState;
    }

    public ConveyorState getConveyorState() {
        return mCurrentConveyorState;
    }

    public void setShooterState(ShooterState state) {
        mCurrentShooterState = state;
    }

    public void setConveyorState(ConveyorState state) {
        mCurrentConveyorState = state;
        System.out.println("Conveyor state: " + mCurrentConveyorState);
    }

    // diff is the reduction in speed for the follower motor
    public void setShooterManualPctg(double pctg, double diff) {
        pctg = MathUtil.clamp(pctg, -1, 1);
        mPIDControllerLead.setReference(pctg, ControlType.kDutyCycle);
        mPIDControllerFollow.setReference(-(pctg - diff), ControlType.kDutyCycle);
        mCurrentShooterState = ShooterState.MANUAL;

    }

    public Command setShooterStateCommand(ShooterState state) {
        return Commands.runOnce(() -> {
            setShooterState(state);
        });
    }

    public Command setConveyorStateCommand(ConveyorState state) {
        return Commands.runOnce(() -> {
            setConveyorState(state);
        });
    }

    private void shooterOff() {
        // mShooterMotor.set(kOffSpeed);
        // mShooterFollowMotor.set(-kOffSpeed);

        mPIDControllerLead.setReference(kOffSpeed, ControlType.kDutyCycle);
        mPIDControllerFollow.setReference(kOffSpeed, ControlType.kDutyCycle);

    }

    private void shooterOn() {
        // mShooterMotor.set(kShootSpeed);
        // mShooterFollowMotor.set(-kShootSpeed);

        mPIDControllerLead.setReference(ON_SPEED + (RobotContainer.addTad ? RobotContainer.britishTad : 0.0), ControlType.kDutyCycle);
        mPIDControllerFollow.setReference(-(ON_SPEED + (RobotContainer.addTad ? RobotContainer.britishTad : 0.0) - 0.05), ControlType.kDutyCycle);
        // mShooterMotor.set(ON_SPEED);
        // mShooterFollowMotor.set(-ON_SPEED + 0.05);
    }

    private void shooterBack() {
        mShooterMotor.set(BACK_SPEED);
        mShooterFollowMotor.set(-BACK_SPEED + 0.05);
    }

    private void conveyorIn() {
        mConveyorMotor.set(kConveyorInSpeed);
    }

    private void conveyorOut() {
        mConveyorMotor.set(kConveyorOutSpeed);
    }

    private void conveyorOff() {
        mConveyorMotor.set(0);
    }

    public boolean hasSpunUp() {
        return (Math.abs(mShooterEncoder.getVelocity()) >= kTargetRPM)
                && (Math.abs(mShooterFollowMotor.getEncoder().getVelocity()) >= kTargetRPM);
    }

    private final DoubleArrayPublisher kCurrent = NetworkTableInstance.getDefault().getDoubleArrayTopic("Currents")
            .publish();

    @Override
    public void updateShuffleboard() {
        mShooterStateWidget.setString(mCurrentShooterState.name());
        mConveyerStateWidget.setString(mCurrentConveyorState.name());
        mShooterSpeedWidget.setDouble(mShooterEncoder.getVelocity());
        mShooterFollowSpeedWidget.setDouble(mShooterFollowMotor.getEncoder().getVelocity());
        kCurrent.accept(new double[] {
                mShooterMotor.getOutputCurrent(),
                mShooterFollowMotor.getOutputCurrent()
        });
    }

    @Override
    public void periodic() {
        switch (mCurrentShooterState) {
            case NONE:
                shooterOff();
                break;
            case OFF:
                shooterOff();
                break;
            case ON:
                shooterOn();
                break;
            case BACK:
                shooterBack();
                break;
            case MANUAL:
                break;

        }
        switch (mCurrentConveyorState) {
            case IN:
            case SHOOTING:
                conveyorIn();
                break;
            case NONE:
                conveyorOff();
                break;
            case STORED:
            case OFF:
                conveyorOff();
                break;
            case OUT:
                conveyorOut();
                break;
            case MANUAL:
                break;
            default:
                break;

        }
        updateShuffleboard();

    }

    @Override
    public void modeSwitchAction() {
        mCurrentShooterState = ShooterState.OFF;
        mCurrentConveyorState = ConveyorState.OFF;

    }

}
