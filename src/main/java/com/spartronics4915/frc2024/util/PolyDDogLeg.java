package com.spartronics4915.frc2024.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;


import org.ddogleg.solver.Polynomial;
import org.ddogleg.solver.PolynomialOps;
import org.ddogleg.solver.PolynomialRoots;
import org.ddogleg.solver.PolynomialSolver;
import org.ddogleg.solver.RootFinderType;
import org.ejml.data.Complex_F64;

public class PolyDDogLeg {
    public static Optional<ArrayList<Double>> getRoots(double a, double b, double c, double d, double e){
        PolynomialRoots finder = PolynomialOps.createRootFinder(5, RootFinderType.EVD);

        Polynomial poly = Polynomial.wrap(a,b,c,d,e);

        if( !finder.process(poly) )
            return Optional.empty();

        List<Complex_F64> roots = finder.getRoots();

        ArrayList<Double> list = new ArrayList<>();

        for( Complex_F64 root : roots ) {
            if( !root.isReal() ) {
                // System.out.println("root is imaginary: "+root);
                continue;
            }

            list.add(root.getReal());
            // double value = poly.evaluate(root.real);
            // System.out.println("Polynomial value at "+root.real+" is "+value);
        }

        return Optional.of(list);
    }

    
}
