package com.spartronics4915.frc2024.util;

import static java.util.stream.Collectors.toList;

import java.util.List;
import java.util.LinkedList;
import java.util.Optional;

import static com.spartronics4915.frc2024.Constants.AutoAimConstants.*;

import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point;

import edu.wpi.first.wpilibj.Timer;

import org.opencv.core.Core;


public class PolynomialRootFinder {

    static{
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    /**
     * coefficients go in order of lowest exponent to highest exponent
     * @param coeffs
     * @return
     */
    public static Optional<List<Double>> getRealRoots(double...coeffs){
        var outMat = new Mat(0, coeffs.length, CvType.CV_16FC2);
        Core.solvePoly(
            new MatOfDouble(coeffs),
            outMat,
            kIterations
        );

        LinkedList<Double> outList = new LinkedList<>();
        // System.out.println(outMat.width());
        // System.out.println(outMat.height());
        for (int i = 0; i < outMat.width(); i++) {
            for (int j = 0; j < outMat.height(); j++) {
                var num = outMat.get(j, i);
                // System.out.println(num[0] + "+" + num[1] + "i");
                if (Math.abs(num[1]) < 1e-10) {
                    outList.add(num[0]);
                }
            }
        }
        return Optional.of(outList);
    }
}
