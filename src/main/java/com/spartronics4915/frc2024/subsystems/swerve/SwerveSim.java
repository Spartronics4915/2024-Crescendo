package com.spartronics4915.frc2024.subsystems.swerve;

import static com.spartronics4915.frc2024.Constants.AutoAimConstants.kAutoAimTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

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

        var obj = field.getObject("target");
        obj.setPose(new Pose2d(kAutoAimTarget.toTranslation2d(), new Rotation2d(0)));
    }

    public Field2d getField() {
        return field;
    }

    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getTable("simStuff").getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    StructPublisher<Pose3d> targetPublisher = NetworkTableInstance.getDefault().getTable("simStuff").getStructTopic("Target", Pose3d.struct).publish();
    StructPublisher<Pose3d> cameraOverridePub = NetworkTableInstance.getDefault().getTable("simStuff").getStructTopic("cameraOverride", Pose3d.struct).publish();
    
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
                m.setPosition(newPos);
            } else {

                validStates = false;
            }
        }

        var statesList = swerveDrive.getModuleDesiredStates();

        if (validStates) {

            ChassisSpeeds currChassisSpeed = kinematics.toChassisSpeeds(statesList);

            Pigeon2SimState simState = IMU.getSimState();
            simState.addYaw(Rotation2d.fromRadians(currChassisSpeed.omegaRadiansPerSecond).getDegrees() * dT);

        }

        publisher.accept(statesList);
        targetPublisher.accept(new Pose3d(kAutoAimTarget, new Rotation3d()));

        Pose2d currPose = swerveDrive.getPose();
        field.setRobotPose(currPose);

        cameraOverridePub.accept(new Pose3d(-0.75, 1.95 + (1.8)*1, 1.5, new Rotation3d(0, Rotation2d.fromDegrees(10).getRadians(), 0)));
    }

}
