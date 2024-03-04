package com.spartronics4915.frc2024.commands.visionauto;

import com.spartronics4915.frc2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveToPickUpNote extends Command {

    SwerveDrive swerveDrive;
    NoteLocatorInterface noteLocator;
    boolean cancelExecution;

    public DriveToPickUpNote(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        if (RobotBase.isSimulation()) {

            noteLocator = new NoteLocatorSim(this.swerveDrive);
        } else {

            noteLocator = new NoteLocatorLimeLight();
        }
        cancelExecution = false;
    }

    @Override
    public void execute() {
        var detectionResult = noteLocator.getClosestVisibleNote();

        if (!detectionResult.isPresent()) {
            cancelExecution = true;
            return;
        }

        NoteLocatorSim.NoteDetection detection = detectionResult.get();
        double vy = 0;
        double vx = 0.5;
        double omega = Rotation2d.fromDegrees(detection.tx() * -1.5).getRadians();
        SmartDashboard.putNumber("estimatedDistance", detection.estimatedDistance());
        ChassisSpeeds driveSpeed;
        System.out.println(detection);
        if (detection.estimatedDistance() < 0.1) {
            driveSpeed = new ChassisSpeeds(0, 0, 0);
            cancelExecution = true;
        } else {

            driveSpeed = new ChassisSpeeds(vx, vy, omega);
        }
        // swerveDrive.drive(driveSpeed, false);
    }

    @Override
    public boolean isFinished() {
        return cancelExecution;
    }

}

// Helper functions
