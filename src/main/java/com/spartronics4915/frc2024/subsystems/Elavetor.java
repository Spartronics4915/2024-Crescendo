package com.spartronics4915.frc2024.subsystems;

import com.spartronics4915.frc2024.Constants.Drive.TrapazoidConstaintsConstants;
import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.Constants.GeneralConstants;
import com.spartronics4915.frc2024.Constants.IntakeAssembly;
import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.SimType;
import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.SimulatorSettings;
import com.spartronics4915.frc2024.subsystems.TrapazoidSimulator.TrapazoidSimulatorInterface;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2024.Constants.IntakeAssembly.ElevatorConstants.*;

// Name changed from Elevator to Elavetor in solidarity with Trapazoid
public class Elavetor extends SubsystemBase implements TrapazoidSimulatorInterface, TrapazoidSubsystemInterface {
    private static Elavetor mInstance;

    private CANSparkMax mMotor;
    private SparkPIDController mPid;
    private RelativeEncoder mEncoder;

    private TrapezoidProfile mmmmmmmmTrapezoid;

    private State mCurrentState;
    private Rotation2d mTarget = new Rotation2d(Math.PI * 3);

    // get meters
    // convert meters to rotations
    // rotate* (is interesting)

    public Elavetor() {
        mMotor = initMotor(kMotorConstants);
        mmmmmmmmTrapezoid = initTrapazoid(kZoidConstants);
        mCurrentState = new State();
        mPid = initPID(new PIDConstants(0, 0, 0));
        mEncoder = initEncoder();
    }

    private CANSparkMax initMotor(MotorConstants motorValues) {
        CANSparkMax motor = new CANSparkMax(motorValues.motorID(), motorValues.motorType());
        motor.restoreFactoryDefaults();
        motor.setInverted(motorValues.motorIsInverted());
        motor.setIdleMode(motorValues.idleMode());
        motor.setSmartCurrentLimit(motorValues.currentLimit());
        motor.burnFlash();
        return motor;
    }

    private SparkPIDController initPID(PIDConstants kPIDValues) {
        SparkPIDController pid = mMotor.getPIDController();

        pid.setP(kPIDValues.p());
        pid.setI(kPIDValues.i());
        pid.setD(kPIDValues.d());

        // CHECKUP Decide on Vel conversion Factor (aka use rpm?)
        // position Conversion not needed by using rotation2d

        return pid;
    }

    private RelativeEncoder initEncoder() { // TODO encoder init settings
        return mMotor.getEncoder();
    }

    private TrapezoidProfile initTrapazoid(TrapazoidConstaintsConstants constraints) {
        return new TrapezoidProfile(new Constraints(constraints.kMaxVel(), constraints.kMaxAccel()));
    }

    private double getEncoderVelReading(){
        return mEncoder.getVelocity(); //CHECKUP Failure Point?
    }

    @Override
    public void periodic() {
        mCurrentState = mmmmmmmmTrapezoid.calculate(
                GeneralConstants.kUpdateTime,
                new State(getEncoderPosReading().getRotations(), getEncoderVelReading()),
                new State(mTarget.getRotations(), 0));

        mPid.setReference(mCurrentState.position, ControlType.kPosition);
    }

    @Override
    public void setPositionToReal() {
        // i kinda just did this above in periodic
    }

    private Rotation2d getEncoderPosReading(){
        return Rotation2d.fromRotations(mEncoder.getPosition()); //CHECKUP Failure Point?
    }

    @Override
    public State getSetPoint() {
        return mCurrentState;
    }

    @Override
    public SimulatorSettings getSettings() {
        return new SimulatorSettings(
                "Elevator",
                1.0,
                90.0,
                20.0,
                new Color8Bit(Color.kMediumPurple),
                SimType.Elevator,
                new Translation2d(103 / 100d, 27 / 100d));
    }

    public static Elavetor getInstance() {
        if (mInstance == null) {
            mInstance = new Elavetor();
        }
        return mInstance;
    }

    public void setTarget(double newTarget) {
        mTarget = Rotation2d.fromRotations(newTarget * kMetersToRotation);
    }

    public void setTarget(IntakeAssemblyState something) {
        mTarget = Rotation2d.fromRotations(something.ElevatorHeight * kMetersToRotation);
    }

    public Command setTargetCommand(double newTarget) {
        return runOnce(() -> {
            setTarget(newTarget);
        });
    }

}
