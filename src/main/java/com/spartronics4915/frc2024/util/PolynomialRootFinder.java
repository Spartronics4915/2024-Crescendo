package com.spartronics4915.frc2024.util;

import static java.util.stream.Collectors.toList;

import java.util.List;
import java.util.Optional;

import org.ddogleg.solver.Polynomial;
import org.ddogleg.solver.PolynomialOps;
import org.ddogleg.solver.PolynomialRoots;
import org.ddogleg.solver.RootFinderType;
import static com.spartronics4915.frc2024.Constants.AutoAimConstants.*;

public class PolynomialRootFinder {
    //using the ddogleg library: website: https://ddogleg.org/  
    public static PolynomialRoots finder = PolynomialOps.createRootFinder(6, kRootFinderType);

    public static Optional<List<Double>> getRealRoots(Polynomial poly){
        try {
            if( !finder.process(poly) )
                return Optional.empty();

            var list = finder.getRoots()
                .stream()
                .filter((r) -> r.isReal())
                .map((r) -> r.real)
                .collect(toList());
            
            return Optional.of(list);
        } catch (Exception e) {
            return Optional.empty();
        }
    }
}
