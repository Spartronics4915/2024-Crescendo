// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2024;

import com.spartronics4915.frc2024.subsystems.ShooterWrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import static edu.wpi.first.units.Units.Degrees;

public final class Main {
    private Main() {}

    public static void main(String... args) {

        // var x = 1;
        // var y = 1;


        // System.out.println(Rotation2d.fromDegrees(ShooterWrist.findAngle(x, y).in(Degrees))); //expected: 45
        // System.out.println(Rotation2d.fromDegrees(ShooterWrist.findAngle(x, -y).in(Degrees))); //expected: -45
        // System.out.println(Rotation2d.fromDegrees(ShooterWrist.findAngle(-x, y).in(Degrees))); //expected: 135
        // System.out.println(Rotation2d.fromDegrees(ShooterWrist.findAngle(-x, -y).in(Degrees))); //expected: -135

        

        RobotBase.startRobot(Robot::new);
    }
}
