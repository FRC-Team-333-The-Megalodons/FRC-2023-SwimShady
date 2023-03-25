package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.Joystick;

public class ColorSensor {
  Joystick joy;
  LEDStrip strip;

  final int NUMBER_OF_LEDS = 285;

  public ColorSensor(){
    strip = new LEDStrip(6, NUMBER_OF_LEDS);
    joy = new Joystick(0);
  }
  
  public void periodic(){
    strip.set(100, 0, 0);  
  }
}