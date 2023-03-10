package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LEDStrip.FancyLED;


public class ColorSensor {
  ColorSensorV3 cs;
  Joystick joy;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  LEDStrip strip = new LEDStrip(6, 120);

  public ColorSensor(){
    cs = new ColorSensorV3(i2cPort);
    joy = new Joystick(1);
  }
   
  public boolean isCube(){
    boolean isCube = false;
    if((cs.getBlue() > 6000 && cs.getBlue() < 8888) && (cs.getRed() > 2000  && cs.getRed() < 3000)
    || (cs.getColor().toString()=="#33507A")){
      isCube = true;
    } else{
      isCube = false;
    }
    return isCube;
  }

  public boolean isCone(){
    boolean isCone = false;
    if ((cs.getBlue() > 2000 && cs.getBlue() < 3500) && (cs.getRed() > 15000 && cs.getRed() < 16500)
    || (cs.getColor().toString()=="#5F8FOF")){
      isCone = true;
    } else{
      isCone = false;
    }
    return isCone;
  }
  
 

  public void periodic(){
    SmartDashboard.putBoolean("Is Cone?", isCone());
    SmartDashboard.putBoolean("Is Cube?", isCube());
    SmartDashboard.putString("Color", cs.getColor().toString());
    SmartDashboard.putNumber("Blue", cs.getBlue());
    SmartDashboard.putNumber("Red", cs.getRed());
    strip.periodic();

    if (isCone()) {
      strip.blink(255, 69, 0);
    }else if(isCube()){
      strip.blink(106, 13, 173);
    }else{
      strip.resetDurationRun();
      strip.resetBlinkRun();
      strip.setFancyDualLayer(FancyLED.ONE_ON_ONE, 0, 0, 100, 100, 0, 0);
    }
  }
}
