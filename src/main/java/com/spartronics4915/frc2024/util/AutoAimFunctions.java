package com.spartronics4915.frc2024.util;

import java.util.Optional;

import static com.spartronics4915.frc2024.Constants.AutoAimConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.*;
public class AutoAimFunctions {

    private static double pow(double a){return Math.pow(a, 2);}
    private static boolean check(double a) {return Double.isFinite(a) && a > 0;}

    /**
     * @param robotPose robots position on the field
     * @param velocity the robots velocity on the field
     * @param Target the targets position on the field
     * @return Translation3d representing target position at collision
     * 
     * based on this equation: 
     * https://www.wolframalpha.com/input?i2d=true&i=solve+Power%5B%7C%7C%7BSubscript%5Bp%2Cx%5D%2CSubscript%5Bp%2Cy%5D%2CSubscript%5Bp%2Cz%5D%7D%2B%7BSubscript%5Bu%2Cx%5D%2CSubscript%5Bu%2Cy%5D%2CSubscript%5Bu%2Cz%5D%7D*Subscript%5Bv%2Cs%5D*Subscript%5Bt%2Cc%5D%7C%7C%2C2%5D%3DPower%5B%5C%2840%29Subscript%5Bv%2Cn%5D*Subscript%5Bt%2Cc%5D%5C%2841%29%2C2%5D++for+Subscript%5Bt%2Cc%5D
     */
    public static Optional<Translation3d> movingAutoAim2(
        Pose2d robotPose, 
        ChassisSpeeds velocity, 
        Translation2d targetPos, 
        double targetZ
    ){
        var p = targetPos.minus(robotPose.getTranslation()); //position of target relative to robot
        var pz = targetZ - kShooterHeight;

        var TempVelocity = new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
        var u = TempVelocity.div(TempVelocity.getNorm());//.times(-1.0); //reversing the direction of the unit velocity vector //no z component
        var vs = TempVelocity.getNorm(); //speaker velocity
        var vn = kShooterSpeed;
        var uz = 0;

        if (vs == 0.0) {
            return Optional.of(new Translation3d(targetPos.getX(), targetPos.getY(), targetZ));
        }
        

        var sqrtPart = Math.sqrt(
            pow(vs) * pow(-2 * p.getX() * u.getX() - 2 * p.getY() * u.getY() - 2 * pz * uz) - 
            4 * (-pow(p.getX()) - pow(p.getY()) - pow(pz)) * (pow(vn) - pow(vs) * pow(u.getX()) - pow(vs) * pow(u.getY()) - pow(vs) * pow(uz))
        );

        var t1 = 
            ((-1) * vs * (-2 * p.getX() * u.getX() - 2 * p.getY() * u.getY() - 2 * pz * uz) - sqrtPart) / 
            (2 * (pow(vn) - pow(vs)*pow(u.getX()) - pow(vs)*pow(u.getY()) - pow(vs)*pow(uz)));

        var t2 = 
            ((-1) * vs * (-2 * p.getX() * u.getX() - 2 * p.getY() * u.getY() - 2 * pz * uz) + sqrtPart) / 
            (2 * (pow(vn) - pow(vs)*pow(u.getX()) - pow(vs)*pow(u.getY()) - pow(vs)*pow(uz)));

        double t = 0.0;
        if (!check(t1) && !check(t2)) {
            return Optional.empty();
        }else if (!check(t1) || !check(t2)){
            t = check(t1) ? t1 : t2;
        }else{
            t = Math.min(t1, t2);
        }

        Translation2d outXY = p.plus(TempVelocity.times(t));
        Translation3d out = new Translation3d(outXY.getX(), outXY.getY(), targetZ);

        if (out.getNorm() > kMaxDistance) {
            return Optional.empty();
        }

        //TODO make sure you understand this
        return Optional.of(out); //relative to robot
    }

    public static Optional<Translation2d> movingAutoAim(
        Translation2d speakPos, //S0
        Translation2d speakVel //V0
    ){
        //the inputs are intended to be relative to the robot in terms of position and velocity, (ie if the robot was moving towards the speaker the speaker would have a velocity going towards the origin)
        //uses code from: https://stackoverflow.com/a/22117046/21621189, look in comments for errors and edge cases accounted for

        //a desmos visualization: https://www.desmos.com/calculator/ejg6jrsodq

        //FIXME account with vertical motion 


        double a = pow(speakVel.getX()) + pow(speakVel.getY()) - pow(kShooterSpeed);
        double b = 2 * ((speakPos.getX() * speakVel.getX()) + (speakPos.getY() * speakVel.getY()) /*- (P1.getX() * speakVel.getX()) - (P1.getY() * speakVel.getY())*/);
        double c = pow(speakPos.getX()) + pow(speakPos.getY()); /*+ (P1.x * P1.x) + (P1.y * P1.y) - (2 * P1.x * P0.x) - (2 * P1.y * P0.y)*/;
        
        if (a == 0) return Optional.empty();
        
        double t1 = (-b + Math.sqrt((b * b) - (4 * a * c))) / (2 * a);
        double t2 = (-b - Math.sqrt((b * b) - (4 * a * c))) / (2 * a);

        double t; //t = time at collision (from 0) in time unit of speakVel 

        if (!check(t1) && !check(t2)) {
            return Optional.empty();
        }else if (!check(t1) || !check(t2)){
            t = check(t1) ? t1 : t2;
        }else{
            t = Math.min(t1, t2);
        }

        Translation2d out = speakPos.plus(speakVel.times(t));

        return Optional.of(out); //outputs the speakerPos at the collision time relative to the robot, (rotation is based on the same as the input translate2ds)
    }

    public static boolean isSafeShot(Translation2d target){
        return (target.getNorm() > kMaxDistance);
    }

    public static Rotation3d getShooterAngleUnsafe(Translation3d targetPos){ //this is the position of the speaker centered around the robot
        double kRelHeight = targetPos.getZ() - kShooterHeight;
        double dist = targetPos.getNorm();

        return new Rotation3d(0, 
            Math.asin(kRelHeight/dist), 
            targetPos.toTranslation2d().getAngle().getRadians()
        );
    }
}
