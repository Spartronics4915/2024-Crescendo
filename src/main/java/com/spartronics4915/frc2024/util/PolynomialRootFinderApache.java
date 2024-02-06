package com.spartronics4915.frc2024.util;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;

import java.util.ArrayList;

public class PolynomialRootFinderApache {
    
    
    public static Double[] findRoots(double a, double b, double c, double d, double e){
        PolynomialFunction polynomial = new PolynomialFunction(new double[]{a,b,c,d,e});
        LaguerreSolver laguerreSolver = new LaguerreSolver();
        
        var list = laguerreSolver.solveAllComplex(new double[]{a,b,c,d,e}, 0);

        ArrayList<Double> realRoots = new ArrayList<>();

        for (var root : list) {
            if (root.getImaginary() == 0) {
                realRoots.add(root.getReal());
            }
        }

        return realRoots.toArray(new Double[]{});
    }
}
