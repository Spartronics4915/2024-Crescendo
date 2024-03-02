package com.spartronics4915.frc2024.util;

import java.util.ArrayList;

public interface ModeSwitchInterface {
    public static ArrayList<ModeSwitchInterface> ModeSwitchSubsystems = new ArrayList<>();

    /**
     * this is a method that when the subsystem is added to {list tbd} when it is disabled or enabled this function will be called
     * you should update your trapezoid motion profiles to the current real position of the motors so that when the subsystem is reeenabled the PIDs don't freak out 
     */
    public void modeSwitchAction();

}
