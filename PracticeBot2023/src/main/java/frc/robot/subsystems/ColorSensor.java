package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDStrip.FancyLED;


public class ColorSensor {
  ColorSensorV3 cs;
  ColorMatch m_colorMatcher;
  Joystick joy;
  LEDStrip strip;

  final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  final int NUMBER_OF_LEDS = 85;

  public ColorSensor(){

    try {
      //cs = new ColorSensorV3(i2cPort);
    } catch (Exception e) {
      // Failed to instantiate color sensor, but it's not fatal for the robot.
      // Just do null checks everywhere in the code so that we don't actually
      //  throw exceptions on failures.
    }
    
    try {
      strip = new LEDStrip(6, NUMBER_OF_LEDS);
    } catch (Exception e) {
      // Failed to instantiate LED Strip, but it's not fatal for the robot.
      // Just do null checks everywhere in the code so that we don't actually
      //  throw exceptions on failures.
    }

    joy = new Joystick(0);
  }

  /*
  // Older version before we learned about the color matcher
  public boolean isCube_old(){
    if (cs == null) { return false; }

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
    if (cs == null) { return false; }

    boolean isCone = false;
    if ((cs.getBlue() > 540 && cs.getBlue() < 3100) && (cs.getRed() > 270 && cs.getRed() < 12700)
    || (cs.getColor().toString()=="#5D8819")){
      isCone = true;
    } else{
      isCone = false;
    }
    return isCone;
  }
  */

  
  public boolean isColor(Color targetColor)
  {
    if (cs == null) { return false; }

    Color detectedColor = cs.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    return match.color == targetColor;
  }

  public boolean isCube()
  {
    return isColor(kBlueTarget);
  }

  public boolean isCone()
  {
    return isColor(kYellowTarget);
  }

  public String getDisplayColor()
  {
    if (cs == null) { return "NULL"; }
    
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

  

  public void periodic(){
    //Timer timer = new Timer();
    //timer.start();
    /* 
    SmartDashboard.putBoolean("Is Cone?", isCone());
    SmartDashboard.putBoolean("Is Cube?", isCube());
    SmartDashboard.putString("Color", getDisplayColor());
    //timer.stop();
    //System.out.println("COLOR SENSOR TIME"+timer.get());
    // Stop here if LED isn't valid
    if (strip == null) { return; }

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
    */
    
    strip.setFancyDualLayer(FancyLED.KNIGHT_RIDER, 0, 0, 100, 0, 0, 0);    
  }
}