package com.spartronics4915.frc2024.commands.visionauto;

import com.spartronics4915.frc2024.subsystems.ShooterWrist;
import edu.wpi.first.wpilibj2.command.Commands;
public class LinearAutoAim {
    

    final static double COEF_M = 1.4;
    final static double COEF_B = 19.71;


    public static double computeOutputAngle(double ty) {
        double outputAngle = COEF_M * ty + COEF_B;

        return outputAngle;
    }
}
