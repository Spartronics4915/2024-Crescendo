package com.spartronics4915.frc2024.util;

import java.util.Optional;

import com.spartronics4915.frc2024.Constants.AutoAimConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoAimFunctions {

    private static double pow(double a){return Math.pow(a, 2);}
    private static boolean check(double a) {return Double.isFinite(a) && a > 0;}

    public static Optional<Translation2d> movingAutoAim(
        Translation2d speakPos, //S0
        Translation2d speakVel //V0
    ){
        
        //uses code from: https://stackoverflow.com/a/22117046/21621189, look in comments for errors and edge cases accounted for

        double kShooterSpeed = AutoAimConstants.kShooterSpeed; //s1

        double a = pow(speakVel.getX()) + pow(speakVel.getY()) - pow(kShooterSpeed);
        double b = 2 * ((speakPos.getX() * speakVel.getX()) + (speakPos.getY() * speakVel.getY()) /*- (P1.getX() * speakVel.getX()) - (P1.getY() * speakVel.getY())*/);
        double c = pow(speakPos.getX()) + pow(speakPos.getY()); /*+ (P1.x * P1.x) + (P1.y * P1.y) - (2 * P1.x * P0.x) - (2 * P1.y * P0.y)*/;
        
        if (a == 0) return Optional.empty();
        
        double t1 = (-b + Math.sqrt((b * b) - (4 * a * c))) / (2 * a);
        double t2 = (-b - Math.sqrt((b * b) - (4 * a * c))) / (2 * a);

        double t; //t = time at collision (from 0) in time unit of speakerVel 

        if (!check(t1) && !check(t2)) {
            return Optional.empty();
        }else if (!check(t1) || !check(t2)){
            t = check(t1) ? t1 : t2;
        }else{
            t = Math.min(t1, t2);
        }

        Translation2d out = speakPos.plus(speakVel.times(t));

        return Optional.of(out); //outputs the speakerPos relative to the robot, rotation relative to the field
    }

    public static Rotation3d getShooterAngle(Translation2d speakPos){ //this is the position of the speaker centered around the robot
        double kRelSpeakHeight = AutoAimConstants.kRelSpeakHeight;
        double dist = speakPos.getNorm();

        return new Rotation3d(0, 
            Math.asin(kRelSpeakHeight/dist), 
            speakPos.getAngle().getRadians()
        );
    }
}
