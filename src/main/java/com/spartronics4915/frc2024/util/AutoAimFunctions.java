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

public class AutoAimFunctions {
    private static double squared(double a) {
        return Math.pow(a, 2);
    }

    /**
     * based on this white paper and using an external library "ddogleg" to find the roots of the polynomial
     * https://docs.google.com/document/d/1TKhiXzLMHVjDPX3a3U0uMvaiW1jWQWUmYpICjIDeMSA/edit
     * @param robotPose
     * @param velocity
     * @param targetPos
     * @param targetZ
     * @return an aiming point for the shooter, relative to the shooter on the robot
     */
    public static Optional<Translation3d> movingAutoAim(
            final Pose2d robotPose, // m, field
            final ChassisSpeeds velocity, // m/s, field
            final Translation3d targetPos // m, field
    ) {
        // varaiable declarations so they match the formulas

        var p = targetPos.toTranslation2d().minus(robotPose.getTranslation());

        double px = p.getX();
        double py = p.getY();
        double pz = targetPos.getZ() - kShooterHeight;

        var v = new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);

        double vx = -v.getX();
        double vy = -v.getY();
        double vz = 0;

        double ax = 0;
        double ay = 0;
        double az = 0 - kGravity; // relative to the robot the speaker will be accelerating upwards

        double s = kShooterSpeed;

        // calculating the coefficients, the number is the exponent

        var c4 = (squared(ax) + squared(ay) + squared(az)) / 4.0;
        var c3 = ax * vx + ay * vy + az * vz;
        var c2 = squared(vx) + px * ax + squared(vy) + py * ay + squared(vz) + pz * az - squared(s);
        var c1 = 2 * (px * vx + py * vy + pz * vz);
        var c0 = squared(px) + squared(py) + squared(pz);

        // get real, non negative roots:

        var possibleRoots = PolynomialRootFinder.getRealRoots(c0, c1, c2, c3, c4);
        if (!possibleRoots.isPresent()) {
            return Optional.empty();
        }
        var realRootList = possibleRoots.get();
        realRootList.removeIf((r) -> r < 0); // remove collision times before the present

        if (realRootList.size() <= 0) {
            return Optional.empty();
        }
        double tc = realRootList.get(0); // earliest collision time
        for (int i = 0; i < Math.min(realRootList.size(), 5); i++) {
            var r = realRootList.get(i);
            if (r > 0) {
                tc = Math.min(r, tc);
            }
        }

        // translate collision time (tc) to target position

        Translation3d out = new Translation3d(px, py, pz)
                .plus(new Translation3d(vx, vy, vz).times(tc))
                .plus(new Translation3d(ax, ay, az).times(squared(tc) * 0.5));

        // if (out.getNorm() > kMaxDistance) {
        // return Optional.empty();
        // }

        // System.out.println(out.div(out.getNorm()).times(kShooterSpeed));

        return Optional.of(out);
    }

    public static Rotation2d getShooterAngle(Translation3d targetPos) { // this is the position of the speaker centered
                                                                        // around the shooter
        // opposite
        double kRelHeight = targetPos.getZ();//- kShooterHeight;
        //hypotenuse
        double dist = targetPos.getNorm();
        
        // sin = o/h
        return new Rotation2d(Math.asin(kRelHeight / dist));
    }

    public static Rotation2d getChassisAngle(Translation3d targetPos) {
        var r = targetPos.toTranslation2d().getAngle().getRotations();
        return Rotation2d.fromRotations(((r < 0) ? 1 + r : r) + 0.5); //TODO is reversing dependent on alliance? 
    }
}
