package com.spartronics4915.frc2024.subsystems;

import com.spartronics4915.frc2024.Constants.IntakeAssembly.IntakeAssemblyState;
import com.spartronics4915.frc2024.Constants.GeneralConstants;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.SimulatorSettings;
import com.spartronics4915.frc2024.subsystems.TrapezoidSimulator.TrapezoidSimulatorInterface;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.spartronics4915.frc2024.util.TrapezoidSubsystemInterface;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2024.Constants.IntakeAssembly.ElevatorConstants.*;

public class Elevator extends SubsystemBase implements TrapezoidSimulatorInterface, TrapezoidSubsystemInterface {
    private static Elevator mInstance;

    private CANSparkMax mMotor;
    private SparkPIDController mPid;
    private RelativeEncoder mEncoder;

    private TrapezoidProfile mmmmmmmmmTrapezoid;

    private State mCurrentState;
    private Rotation2d mTarget = new Rotation2d(Math.PI * 3);

    // get meters
    // convert meters to rotations
    // rotate* (is interesting)

    public Elevator() {
        // Initializes the motor
        mMotor = new CANSparkMax(kMotorConstants.motorID(), kMotorConstants.motorType());
        mMotor.restoreFactoryDefaults();
        mMotor.setInverted(kMotorConstants.motorIsInverted());
        mMotor.setIdleMode(kMotorConstants.idleMode());
        mMotor.setSmartCurrentLimit(kMotorConstants.currentLimit());
        mMotor.burnFlash();

        // Initializes the Trapezoid
        mmmmmmmmmTrapezoid = new TrapezoidProfile(kZoidConstants);

        // Sets the current state
        mCurrentState = new State();

        // Initializes the PID
        mPid = mMotor.getPIDController();
        mPid.setP(0);
        mPid.setI(0);
        mPid.setD(0);
        // CHECKUP Decide on Vel conversion Factor (aka use rpm?)

        // Sets up the encoder
        mEncoder = mMotor.getEncoder();
    }

    private double getEncoderVelReading(){
        return mEncoder.getVelocity(); //CHECKUP Failure Point?
    }
    private Rotation2d getEncoderPosReading() {
        return Rotation2d.fromRotations(mEncoder.getPosition()); //CHECKUP Failure Point?
    }

    @Override
    public void periodic() {
        mCurrentState = mmmmmmmmmTrapezoid.calculate(
                GeneralConstants.kUpdateTime,
                new State(getEncoderPosReading().getRotations(), getEncoderVelReading()),
                new State(mTarget.getRotations(), 0));
        mPid.setReference(mCurrentState.position, ControlType.kPosition);
    }

    @Override
    public State getSetPoint() {
        return mCurrentState;
    }

    /**
     * @return Simulator Settings for the Elevator
     */
    @Override
    public SimulatorSettings getSettings() {
        return kElevatorSimulatorSettings;
    }

    /**
     * @return height of elevator
     */
    public double getHeight() {
        return getEncoderPosReading().getRotations() / kMetersToRotation;
    }
    /**
     * @return height of elevator
     */
    public double getDistance() {
        return getHeight();
    }

    /**
     * Sets the new target position
     * @param newTarget New target position (in meters... i think)
     */
    public void setTarget(double newTarget) {
        mTarget = Rotation2d.fromRotations(newTarget * kMetersToRotation);
    }
    /**
     * Sets the new target position
     * @param intakeAssemblyState I don't know what this is but I was told to add it so I did
     */
    public void setTarget(IntakeAssemblyState intakeAssemblyState) {
        mTarget = Rotation2d.fromRotations(intakeAssemblyState.ElevatorHeight * kMetersToRotation);
    }
    /**
     * Command to set the new target position
     * @param newTarget New target position (in meters... i think)
     */
    public Command setTargetCommand(double newTarget) {
        return runOnce(() -> {
            setTarget(newTarget);
        });
    }

    // Here because something requires it
    @Override
    public void setPositionToReal() {}

    /**
     * @return A static instance of the elevator subsystem
     */
    public static Elevator getInstance() { 
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

}
