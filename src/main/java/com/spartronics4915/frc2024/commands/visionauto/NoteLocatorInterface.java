package com.spartronics4915.frc2024.commands.visionauto;

import java.util.Optional;

public interface NoteLocatorInterface {
    public static record NoteDetection(double tx, double ty, double estimatedDistance) {};

    public Optional<NoteDetection> getClosestVisibleNote();
}
