package com.spartronics4915.frc2024.commands.visionauto;

import java.util.Optional;

public interface TargetDetectorInterface {
    
    public static record Detection(double tx, double ty, double estimatedDistance) {};

    public Optional<Detection> getClosestVisibleTarget();

}
