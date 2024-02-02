package com.spartronics4915.frc2024.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Pose2d;
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
        SmartDashboard.putData("field", field);
    }

    public Field2d getField() {
        return field;
    }

    @Override
    public void simulationPeriodic() {

        double dT = Timer.getFPGATimestamp() - lastTime;
        boolean validStates = true;
        Pigeon2 IMU = swerveDrive.getIMU();

        if (lastTime == 0) {
            lastTime = Timer.getFPGATimestamp();
            return;
        }
        else {
            lastTime = Timer.getFPGATimestamp();
        }


        // Now it is time to update the module states
        // We are going to update to

        for (SwerveModule m : swerveModules) {
            // FIXME: No rotation in sim. Might just be a result of using the keyboard as a joystick. 
            // Update the state
            SwerveModuleState currDesiredState = m.getDesiredState();
            if (currDesiredState != null) {
                double newPosDist = m.getPosition().distanceMeters + dT * currDesiredState.speedMetersPerSecond;
                Rotation2d newPosAngle = currDesiredState.angle; // hack but good enough for sim
                var newPos = new SwerveModulePosition(newPosDist, newPosAngle);
                System.err.println(newPos.toString());
                m.setPosition(newPos);
            } else {

                validStates = false;
            }
        }

        if (validStates) {

            ChassisSpeeds currChassisSpeed = kinematics.toChassisSpeeds(swerveDrive.getModuleDesiredStates());

            Pigeon2SimState simState = IMU.getSimState();
            simState.addYaw(Rotation2d.fromRadians(currChassisSpeed.omegaRadiansPerSecond).getDegrees() * dT);

        }

        Pose2d currPose = swerveDrive.getPose();
        field.setRobotPose(currPose);
    }

}
