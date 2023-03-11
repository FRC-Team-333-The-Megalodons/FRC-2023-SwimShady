package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDStrip.FancyLED;


public class ColorSensor {
  ColorSensorV3 cs;
  ColorMatch m_colorMatcher;
  Joystick joy;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  LEDStrip strip;

  final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  public ColorSensor(){
    m_colorMatcher = new ColorMatch();
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    try {
      cs = new ColorSensorV3(i2cPort);
      strip = new LEDStrip(6, 85);
    } catch (Exception e) {
      // Failed to instantiate color sensor, that shouldn't be fatal though.
      // Just do null checks everywhere in the code so that we don't actually
      //  throw exceptions on failures.
    }
    
    joy = new Joystick(1);
  }

  public boolean isSensorValid()
  {
    return cs != null && strip != null;
  }
   
  // Older version before we learned about the color matcher
  public boolean isCube_old(){
    if (!isSensorValid()) { return false; }

    boolean isCube = false;
    if((cs.getBlue() > 230 && cs.getBlue() < 8888) && (cs.getRed() > 140  && cs.getRed() < 3000)
    || (cs.getColor().toString()=="#33507A")){
      isCube = true;
    } else{
      isCube = false;
    }
    return isCube;
  }

  // Older version before we learned about the color matcher
  public boolean isCone_old(){
    if (!isSensorValid()) { return false; }

    boolean isCone = false;
    if ((cs.getBlue() > 540 && cs.getBlue() < 3100) && (cs.getRed() > 270 && cs.getRed() < 12700)
    || (cs.getColor().toString()=="#5D8819")){
      isCone = true;
    } else{
      isCone = false;
    }
    return isCone;
  }

  
  public boolean isColor(Color targetColor)
  {
    Color detectedColor = cs.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    return match.color == targetColor;
  }

  public boolean isCube()
  {
    if (!isSensorValid()) { return false; }

    return isColor(kBlueTarget);
  }

  public boolean isCone()
  {
    if (!isSensorValid()) { return false; }

    return isColor(kYellowTarget);
  }

  public String getDisplayColor()
  {
    Color detectedColor = cs.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      return "Blue";
    } else if (match.color == kGreenTarget) {
      return "Green";
    } else if (match.color == kRedTarget) {
      return "Red";
    } else if (match.color == kYellowTarget) {
      return "Yellow";
    }

    return "Unknown";
  }

  public void resetDashboardValues()
  {
    SmartDashboard.putBoolean("Is Cone?", false);
    SmartDashboard.putBoolean("Is Cube?", false);
    SmartDashboard.putString("Color", "NULL");
  }

  

  public void periodic(){
    if (!isSensorValid()) {
      resetDashboardValues();
      return;
    }

    SmartDashboard.putBoolean("Is Cone?", isCone());
    SmartDashboard.putBoolean("Is Cube?", isCube());
    SmartDashboard.putString("Color", getDisplayColor());
    
    strip.periodic();

    if (isCone()) {
      //strip.blink(255, 69, 0);
      strip.setFancyDualLayer(FancyLED.PULSE, 255, 69, 0, 0,0,0);
    }else if(isCube()){
      //strip.blink(106, 13, 173);
      strip.setFancyDualLayer(FancyLED.PULSE, 106, 13, 173, 0,0,0);
    }else{
      strip.resetDurationRun();
      strip.resetBlinkRun();
      //strip.set(145, 70, 0);//yellow
      //strip.set(10, 0, 40);//purple
      strip.setFancyDualLayer(FancyLED.PULSE, 0, 0, 100, 155, 10, 0);
    }
  }
}