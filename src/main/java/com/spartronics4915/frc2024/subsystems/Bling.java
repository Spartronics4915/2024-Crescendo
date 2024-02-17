// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2024.subsystems;

import java.util.LinkedList;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.spartronics4915.frc2024.Constants;
import com.spartronics4915.frc2024.Constants.BlingModes;

import static com.spartronics4915.frc2024.Constants.BlingConstants.*;

/**
 * Subsystem for Bling
 */
public class Bling extends SubsystemBase {
  //#region variables and such
  private static Bling mInstance;

  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
  private Constants.BlingModes mMode;
  private Color mPrimary;
  private Color mSecondary;
  private int mFrame;
  private boolean mSetSecondary;

  // Fancy Mode List
  private static LinkedList<BlingMCwithPriority> list = new LinkedList<BlingMCwithPriority>();
  public record BlingMCwithPriority(Supplier<Optional<BlingMC>> mc, int priority) {}
  public record BlingMC(BlingModes mode, Color primary, Color secondary) {}

  //#endregion

  /**
   * Constructor for bling subsystem with default settings as set in Constants
   */
  private Bling() {
    mMode = kDefaultBlingMode;
    mPrimary = kDefaultBlingColor;
    mSecondary = kDefaultBlingColorSecondary;

    mSetSecondary = false;
    mFrame = 0;

    led = new AddressableLED(kLedPort);
    led.setLength(kLedLength);

    ledBuffer = new AddressableLEDBuffer(kLedLength);

    led.start();
    shuffleboardFunc();
  }

  //#region periodically

  @Override
  public void simulationPeriodic() {
    mFrame++;
    update();
  }

  @Override
  public void periodic() {
    mFrame++;
    update();
  }

  /**
   * Updates the LEDS.
   */
  public void update() {
    if (kIsInFancyMode) setBling(ughhhhhhhhhhhhh());
    if (kSpam) System.out.println("Bling - " + mMode + ", " + mPrimary + ", " + mSecondary);
    switch (mMode) {
      case OFF -> setAllLeds(0, 0, 0);
      case SOLID -> setAllLeds(mPrimary.red, mPrimary.green, mPrimary.blue);
      case SOLID_SECONDARY -> setAllLeds(mSecondary.red, mSecondary.green, mSecondary.blue);
      case GRADIENT_REVERSED -> {
        for (int i = 0; i < kLedLength; i++) {
          setPixel(i, mix(mPrimary, mSecondary, (float) (i) / kLedLength));
        }
        led.setData(ledBuffer);
      }
      case GRADIENT -> {
        for (int i = 0; i < kLedLength; i++) {
          setPixel(i, mix(mSecondary, mPrimary, (float) (i) / kLedLength));
        }
      }
      case PULSE ->
        setAllLeds(mix(mPrimary, Color.kBlack, Math.abs((float) mFrame % kPulseLength / kPulseLength - .5f) * 2f));
      case PULSE_SWITCH ->
        setAllLeds(mix(mPrimary, mSecondary, Math.abs((float) mFrame % kPulseLength / kPulseLength - .5f) * 2f));
      case AROUND -> {
        for (int i = 0; i < kLedLength; i++) {
          if (kAroundStripLength >= Math.abs(i - (mFrame*kAroundSpeedMultiplier % kLedLength)))
            setPixel(i, mix(Color.kBlack, mPrimary, Math.abs(i - (mFrame*kAroundSpeedMultiplier % kLedLength))/kAroundStripLength));
          else setPixel(i, Color.kBlack);
        }
      }
      case AROUND_SECONDARY_BG -> {
        for (int i = 0; i < kLedLength; i++) {
          if (kAroundStripLength >= Math.abs(i - (mFrame*kAroundSpeedMultiplier % kLedLength)))
            setPixel(i, mix(mSecondary, mPrimary, Math.abs(i - (mFrame*kAroundSpeedMultiplier % kLedLength))/kAroundStripLength));
          else setPixel(i, mSecondary);
        }
      }
      case ERROR ->
        setAllLeds(mix(Color.kRed, Color.kHotPink, Math.abs((float) mFrame % kAlertLength / kAlertLength - .5f) * 2f));
      case WARNING ->
        setAllLeds(mix(Color.kYellow, Color.kOrange, Math.abs((float) mFrame % kAlertLength / kAlertLength - .5f) * 2f));
    }

    led.setData(ledBuffer);
  }

  /**
   * The *fancy* mode stuff
   * @return
   */
  private BlingMC ughhhhhhhhhhhhh() {
    for (BlingMCwithPriority blingMCwithPriority : list) {
      Optional<BlingMC> x = blingMCwithPriority.mc().get();
      if (x.isPresent()) return x.get();
    }
    return new BlingMC(BlingModes.OFF, Color.kBlack, Color.kBlack);
  }

  //#endregion

  public void shuffleboardFunc() {
    ShuffleboardTab tab = Shuffleboard.getTab("bling");
    tab.add("reset", this.resetCommand());
    tab.add("off", this.setModeCommand(BlingModes.OFF));
    tab.add("solid", this.setModeCommand(BlingModes.SOLID));
    tab.add("solid #2", this.setModeCommand(BlingModes.SOLID_SECONDARY));
    tab.add("gradient", this.setModeCommand(BlingModes.GRADIENT));
    tab.add("gradient reversed", this.setModeCommand(BlingModes.GRADIENT_REVERSED));
    tab.add("pulse", this.setModeCommand(BlingModes.PULSE));
    tab.add("pulse switch", this.setModeCommand(BlingModes.PULSE_SWITCH));
    tab.add("around", this.setModeCommand(BlingModes.AROUND));
    tab.add("around w/ #2 bg", this.setModeCommand(BlingModes.AROUND_SECONDARY_BG));

    tab.add("warn", this.setModeCommand(BlingModes.WARNING));
    tab.add("ERROR", this.setModeCommand(BlingModes.ERROR));

    tab.add("set secondary", this.setSecondaryColorCommand());

    tab.add("red", this.setColorCommand(Color.kRed));
    tab.add("orange", this.setColorCommand(Color.kOrange));
    tab.add("yellow", this.setColorCommand(Color.kYellow));
    tab.add("green", this.setColorCommand(Color.kGreen));
    tab.add("blue", this.setColorCommand(Color.kBlue));
    tab.add("cyan", this.setColorCommand(Color.kCyan));
    tab.add("magenta", this.setColorCommand(Color.kMagenta));
    tab.add("white", this.setColorCommand(Color.kWhite));
    tab.add("alliance color", this.setColorCommand(getAllianceColor()));
  }

  //#region Set mode, set bling, set colors, etc

  /**
   * Sets the bling mode
   * @param newMode The new Bling Mode for the lighting
   */
  public void setMode(BlingModes newMode) {
    this.mMode = newMode;
  }
  public Command setModeCommand(BlingModes newMode) {
    return runOnce(() -> setMode(newMode));
  }

  /**
   * Sets the current color, if setSecondary is true then sets secondary color.
   * @param newColor The new bling color
   */
  public void setColor(Color newColor) {
    this.mPrimary = newColor;
  }
  public void setSecondary(Color newColor) {
    this.mSecondary = newColor;
  }
  public Command setColorCommand(Color newColor) {
    return runOnce(() -> {
      if (mSetSecondary) setSecondary(newColor);
      else setColor(newColor);
      mSetSecondary = false;
    });
  }
  public Command setSecondaryColorCommand() {
    return runOnce(() -> mSetSecondary = true);
  }

  public void setBling(BlingMC mc) {
    setMode(mc.mode());
    setColor(mc.primary());
    setSecondary(mc.secondary());
  }

  /**
   * Resets settings such as color and bling mode to defaults
   */
  public void reset() {
      mMode = kDefaultBlingMode;
      mPrimary = kDefaultBlingColor;
      mSecondary = kDefaultBlingColorSecondary;

      mSetSecondary = false;
      mFrame = 0;
  }
  public Command resetCommand() {
    return runOnce(this::reset);
  }

  //#endregion

  //#region Color Functions
  public static Color getAllianceColor() {
    if (DriverStation.getAlliance().isPresent())
      return DriverStation.getAlliance().get().equals(Alliance.Blue) ? Color.kRoyalBlue : Color.kRed;
    return Color.kYellow; // No Color
  }

  /**from www.java2s.com
  * Mixes two colours.
  * @param a       Base colour.
  * @param b       Colour to mix in.
  * @param percent Percentage of the old colour to keep around.
  * @return the mixed Color
  */
   public static Color mix(Color a, Color b, double percent) {
    return new Color((mixNumsForColors(a.red, b.red, percent)),
                     (mixNumsForColors(a.green, b.green, percent)),
                     (mixNumsForColors(a.blue, b.blue, percent)));
  }

  /**
   * Mixes two numbers together with a range of 0-1 for colors.
   * @param a       First number.
   * @param b       Second number.
   * @param percent Percentage of mix.
   * @return Mixed numbers
   */
  private static double mixNumsForColors(double a, double b, double percent) {
    return MathUtil.clamp(a * percent + b * (1.0 - percent),0,1);
  }

  //#endregion

  //#region Set LED or Strip

  /**
   * Sets the color of all the LEDS in the String.
   * @param r The red value of the color
   * @param g The green value of the color
   * @param b The blue value of the color
   */
  private void setAllLeds(double r, double g, double b) {
    for (int i = 0; i < kLedLength; i++) {
      setPixel(i, r, g, b);
    }
  }

  /**
   * Sets the color of all the LEDS in the String.
   * @param c Color to set
   */
  private void setAllLeds(Color c) {
    for (int i = 0; i < kLedLength; i++) {
      setPixel(i, c.red, c.green, c.blue);
    }
  }

  /**
   * Sets the color of a single pixel
   * @param index Index of the pixel
   * @param c Color to set
   */
  private void setPixel(int index, Color c) {
    ledBuffer.setRGB(index, (int) (c.red*kBrightness*255.0), (int) (c.green*kBrightness*255.0), (int) (c.blue*kBrightness*255.0));
  }

  /**
   * Sets the color of a single pixel
   * @param index Index of the pixel
   * @param r Red value of the pixel
   * @param g Green value of the pixel
   * @param b Blue value of the pixel
   */
  private void setPixel(int index, double r, double g, double b) {
    ledBuffer.setRGB(index, (int) (r*kBrightness*255.0), (int) (g*kBrightness*255.0), (int) (b*kBrightness*255.0));
  }

  //#endregion

  /**
  * @return A static instance of the elevator subsystem
  */
  public static Bling getInstance() {
      if (mInstance == null) {
          mInstance = new Bling();
      }
      return mInstance;
  }

  /**
   * Add a BlingMC with priority to the fancy mode list
   * @param add What BlingMCwithPriority to add
   */
  public static void addToLinkedList(BlingMCwithPriority add) {
    list.add(add);
    list.sort((a,b) -> {
      return a.priority() - b.priority();
    });
  }

}
