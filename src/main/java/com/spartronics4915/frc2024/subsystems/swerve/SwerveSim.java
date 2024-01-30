package com.spartronics4915.frc2024.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveSim extends SubsystemBase {

    SwerveDrive swerveDrive;
    SwerveModule[] swerveModules;
    Field2d field;
    SwerveDriveKinematics kinematics;
    double lastTime;

    public SwerveSim(SwerveDrive swerveDrive) {

        this.swerveDrive = swerveDrive;
        kinematics = swerveDrive.getSwerveDriveKinematics();
        field = new Field2d();

        swerveModules = swerveDrive.getSwerveModules();
        lastTime = 0;
    }

    public Field2d getField() {
        return field;
    }

    @Override
    public void simulationPeriodic() {

        Pigeon2 IMU = swerveDrive.getImU();

        if (lastTime == 0) {
            lastTime = Timer.getFPGATimestamp();
            return;
        }

        double dT = Timer.getFPGATimestamp() - lastTime;

        ChassisSpeeds currChassisSpeed = kinematics.toChassisSpeeds(swerveDrive.getModuleStates());

        Pigeon2SimState simState = IMU.getSimState();
        simState.addYaw(Rotation2d.fromRadians(currChassisSpeed.omegaRadiansPerSecond).getDegrees() * dT);

        // Now it is time to update the module states
        // We are going to update to

        for (SwerveModule m : swerveModules) {

            // Update the state
            SwerveModuleState currDesiredState = m.getDesiredState();
            if (currDesiredState != null) {
                double newPos = m.getPosition().distanceMeters + dT * currDesiredState.speedMetersPerSecond;
                m.setPosition(newPos);
            }
        }
    }

}
