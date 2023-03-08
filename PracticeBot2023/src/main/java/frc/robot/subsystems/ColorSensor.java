package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ColorSensor {
  ColorSensorV3 cs;
  AddressableLED LED;
  AddressableLEDBuffer ledBuffer;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private int m_rainbowFirstPixelHue;

  public ColorSensor(){
    cs = new ColorSensorV3(i2cPort);
    LED = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(2000);
    LED.setLength(ledBuffer.getLength());

    LED.setData(ledBuffer);
    LED.start();
  }

  public boolean leds(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledBuffer.setRGB(i, 200, 0, 122);
   }
   
    LED.setData(ledBuffer);
    return false;
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the value
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
  
 /*public boolean isCone(){
    boolean isCone = false;
    if(cs.getBlue() == 4444 && cs.getRed() == 7777 || cs.getColor().toString()=="#5F900F"){
      isCone = true;
    } else{
      isCone = false;
    }
    return isCone;
  }*/
  
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

    // Set the data
    /*for (var i = 0; i < ledBuffer.getLength(); i++) {
     

      ledBuffer.setRGB(i, 200, 0, 122);
    }
   
    LED.setData(ledBuffer);
    //LED.start();*/
    //rainbow();

    if (isCone()) {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        //ledBuffer.setRGB(i, 255, 255, 0);
        ledBuffer.setRGB(i, 0, 255, 0);
      }
    } else if (isCube()) {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 200, 0, 122);
      }
    } else {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 0, 0, 255);
      }
    }
    LED.setData(ledBuffer);
  }
}
