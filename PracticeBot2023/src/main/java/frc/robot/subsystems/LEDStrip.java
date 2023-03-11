// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  /** Creates a new LEDStrip. */
  AddressableLED LED;
  AddressableLEDBuffer ledBuffer;
  int blinkRun, durationRun;
  int blinkRange, blinkMax, durationMax;

  public LEDStrip(int port, int buffer) {
    LED = new AddressableLED(port);//default is 6
    ledBuffer = new AddressableLEDBuffer(buffer);//default is 120
    LED.setLength(ledBuffer.getLength());

    LED.setData(ledBuffer);
    LED.start();

    blinkRun = 0;
    durationRun = 0;

    //default vals for blinking
    blinkRange = 5;
    blinkMax = 10;
    durationMax = 110;
  }

  public LEDStrip(int port, int buffer, int blinkRange, int blinkMax, int durationMax) {
    LED = new AddressableLED(port);//default is 6
    ledBuffer = new AddressableLEDBuffer(buffer);//default is 120
    LED.setLength(ledBuffer.getLength());

    LED.setData(ledBuffer);
    LED.start();

    blinkRun = 0;
    durationRun = 0;

    //default vals for blinking
    this.blinkRange = blinkRange;
    this.blinkMax = blinkMax;
    this.durationMax = durationMax;
  }

  public void blink(int r, int g, int b){
    if(durationRun <= durationMax){
      for(var i = 0; i < ledBuffer.getLength(); i++){
        ledBuffer.setRGB(i, (blinkRun <= blinkRange ? r : 0) , (blinkRun <= blinkRange ? g : 0), (blinkRun <= blinkRange ? b : 0));
      }
    }else{
      for(var i = 0; i < ledBuffer.getLength(); i++){
        ledBuffer.setRGB(i, r,g,b);
      }
    }
  }

  public void set(int r, int g, int b){
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, r, g, b);
    }
  }

  public void setFancy(FancyLED ledMode, int r, int g, int b){
    if(ledMode == FancyLED.ONE_ON_ONE){
      for(var i = 0; i < ledBuffer.getLength(); i++){
        if(i % 2 == 0){
          ledBuffer.setRGB(i, r, g, b);
        }
      }
    }else if(ledMode == FancyLED.KNIGHT_RIDER){

    }else if(ledMode == FancyLED.PULSE){

    }
  }

  int lastI = 0;
  int pulseRun = 0, pulseRange = 50, pulseMax = 100;
  public void setFancyDualLayer(FancyLED ledMode, int r, int g, int b, int r2, int g2, int b2){
    if(ledMode == FancyLED.ONE_ON_ONE){
      for(var i = 0; i < ledBuffer.getLength(); i++){
        if(i % 2 == 0){
          ledBuffer.setRGB(i, r, g, b);
        }else{
          ledBuffer.setRGB(i, r2, g2, b2);
        }
      }
    }else if(ledMode == FancyLED.KNIGHT_RIDER){
      for(var i = 0; i < ledBuffer.getLength(); i++){
        ledBuffer.setRGB(i, r2, g2,b2);
      }
      if(lastI == 140){
        lastI = 0;
      }
      lastI++;
      for(int i = lastI; i < lastI+40; i++){
        ledBuffer.setRGB(i, r,g,b);
      }
    }else if(ledMode == FancyLED.PULSE){
      for(var i = 0; i < ledBuffer.getLength(); i++){
        ledBuffer.setRGB(i, (pulseRun <= pulseRange ? r : 0) , (pulseRun <= pulseRange ? g : 0), (pulseRun <= pulseRange ? b : 0));
      }

      pulseRun++;
      if(pulseRun == pulseMax){
        pulseRun = 0;
      }
    }
  }

  public void resetBlinkRun(){
    blinkRun = 0;
  }

  public void resetDurationRun(){
    durationRun = 0;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ++blinkRun;
    ++durationRun;
    if(blinkRun >= blinkMax){
      blinkRun = 0;
    }
    LED.setData(ledBuffer);
  }

  public static enum FancyLED{
      ONE_ON_ONE,
      KNIGHT_RIDER,
      PULSE;
  }
}
