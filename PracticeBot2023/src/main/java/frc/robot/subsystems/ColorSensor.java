package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.Joystick;

public class ColorSensor {
  Joystick joy;
  LEDStrip strip;

  final int NUMBER_OF_LEDS = 130;

  public ColorSensor(){
    strip = new LEDStrip(0, NUMBER_OF_LEDS);
    joy = new Joystick(0);
  }

  boolean toggle = false;
  
  public void periodic(){
    if(joy.getRawButton(3)){
      strip.set(106, 13,173);
    }else {
      strip.set(255, 215, 0);
    }
  }
}