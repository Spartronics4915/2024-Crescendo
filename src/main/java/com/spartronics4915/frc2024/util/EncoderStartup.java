package com.spartronics4915.frc2024.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;

public interface EncoderStartup {
    public static ArrayList<EncoderStartup> EncoderResetList = new ArrayList<>();

    public static Command startupResetCommand(){
        return new Command() {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }

            @Override
            public void execute() {
                for (var subsys : EncoderResetList) {
                    for (EncoderStartupSettings settings : subsys.getEncoderSettings()) {
                        settings.encoderStartup();
                    }
                }
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    public record EncoderStartupSettings(
        RelativeEncoder encoder,
        double intendedValue
    ) {
        public void encoderStartup(){
            encoder.setPosition(intendedValue);
        }
    }
    
    public EncoderStartupSettings[] getEncoderSettings(); 
}
