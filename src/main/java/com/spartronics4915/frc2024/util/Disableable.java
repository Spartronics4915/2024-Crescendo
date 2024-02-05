package com.spartronics4915.frc2024.util;

import java.util.function.*;
import java.util.Optional;

public interface Disableable {

    abstract void disable();

    public static boolean handleBoolOpt(Optional<Boolean> value, boolean whenOptional){
        return value.isPresent() && value.get() ;
    }
}
