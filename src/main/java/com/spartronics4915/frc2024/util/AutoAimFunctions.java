package com.spartronics4915.frc2024.util;

import java.util.Optional;

import static com.spartronics4915.frc2024.Constants.AutoAimConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.*;

import org.ddogleg.solver.Polynomial;



public class AutoAimFunctions {

    private static double pow(double a){return Math.pow(a, 2);}

    /**
     * based on this white paper and using an external library "ddogleg" to find the roots of the polynomial
     * https://docs.google.com/document/d/1TKhiXzLMHVjDPX3a3U0uMvaiW1jWQWUmYpICjIDeMSA/edit
     * @param robotPose
     * @param velocity
     * @param targetPos
     * @param targetZ
     * @return an aiming point for the shooter
     */
    public static Optional<Translation3d> movingAutoAim(
        final Pose2d robotPose, // m, field
        final ChassisSpeeds velocity, //m/s, field 
        final Translation2d targetPos, //m, field
        final double targetZ //m, ground
    ){
        //varaiable declarations so they match the formulas

        var p = targetPos.minus(robotPose.getTranslation());

        double px = p.getX();
        double py = p.getY();
        double pz = targetZ - kShooterHeight;

        var v = new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);

        double vx = -v.getX();
        double vy = -v.getY();
        double vz = 0;

        double ax = 0;
        double ay = 0;
        double az = kGravity;

        double s = kShooterSpeed;

        //calculating the coefficients, the number is the exponent

        var c4 = (pow(ax) + pow(ay) + pow(az))/4.0;
        var c3 = ax * vx + ay * vy + az * vz;
        var c2 = pow(vx) + px * ax + pow(vy) + py * ay + pow(vz)+ pz * az - pow(s);
        var c1 = 2 * (px * vx + py * vy+pz * vz);
        var c0 = pow(px) + pow(py) + pow(pz);

        // get real, non negative roots:

        var possibleRoots = PolynomialRootFinder.getRealRoots(Polynomial.wrap(c0,c1,c2,c3,c4));
        if (!possibleRoots.isPresent()) {
            return Optional.empty();
        }
        var realRootList = possibleRoots.get();
        realRootList.removeIf((r) -> r > 0); //remove collision times before the present
        
        double tc = realRootList.get(0); //earliest collision time
        for (int i = 0; i < Math.min(realRootList.size(), 5); i++) {
            var r = realRootList.get(i);
            if (r > 0) {
                tc = Math.min(r, tc);
            }
        }

        //translate collision time (tc) to target position

        Translation3d out = 
            new Translation3d(p.getX(), p.getY(), targetZ)
            .plus(new Translation3d(vx, vy, vz).times(tc))
            .plus(new Translation3d(ax, ay, az).times(pow(tc) * 0.5));

        if (out.getNorm() > kMaxDistance) {
            return Optional.empty();
        }

        return Optional.of(out);
    }

    public static Rotation3d getShooterAngle(Translation3d targetPos){ //this is the position of the speaker centered around the robot
        double kRelHeight = targetPos.getZ() - kShooterHeight; //CHECKUP maybe remove the shooterheight part here?
        double dist = targetPos.getNorm();

        return new Rotation3d(0, 
            Math.asin(kRelHeight/dist), 
            targetPos.toTranslation2d().getAngle().getRadians()
        );
    }
}
