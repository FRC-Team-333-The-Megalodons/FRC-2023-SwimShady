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
  long blinkRun, durationRun;
  long blinkRange, blinkMax, durationMax;

  public LEDStrip(int port, int numberOfLeds) {
    LED = new AddressableLED(port);//default is 6
    ledBuffer = new AddressableLEDBuffer(numberOfLeds);//default is 120
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

  public LEDStrip(int port, int numberOfLeds, int blinkRange, int blinkMax, int durationMax) {
    LED = new AddressableLED(port);//default is 6
    ledBuffer = new AddressableLEDBuffer(numberOfLeds);//default is 120
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
      for(int i = 0; i < ledBuffer.getLength(); ++i){
        ledBuffer.setRGB(i, (blinkRun <= blinkRange ? r : 0) , (blinkRun <= blinkRange ? g : 0), (blinkRun <= blinkRange ? b : 0));
      }
    }else{
      for(int i = 0; i < ledBuffer.getLength(); ++i){
        ledBuffer.setRGB(i, r,g,b);
      }
    }
  }

  public void set(int r, int g, int b){
    for(int i = 0; i < ledBuffer.getLength(); ++i){
      ledBuffer.setRGB(i, r, g, b);
    }
    LED.setData(ledBuffer);
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
    LED.setData(ledBuffer);
  }

  int lastI = 0;
  int pulseRun = 0, pulseRange = 70, pulseMax = 140;

  FancyLED m_prev_ledMode;
  int m_prev_r, m_prev_g, m_prev_b, m_prev_r2, m_prev_g2, m_prev_b2;
  boolean m_didLedBufferChange;

  public void setFancyDualLayer(FancyLED ledMode, int r, int g, int b, int r2, int g2, int b2){
    if (m_prev_ledMode == ledMode &&
        m_prev_r == r && m_prev_g == g && m_prev_b == b && 
        m_prev_r2 == r2 && m_prev_g2 == g2 && m_prev_b2 == b2)
    {
      m_didLedBufferChange = false;
      return;
    }

    m_didLedBufferChange = true;


    if(ledMode == FancyLED.ONE_ON_ONE){
      for(int i = 0; i < ledBuffer.getLength(); ++i){
        if(i % 2 == 0){
          ledBuffer.setRGB(i, r, g, b);
        }else{
          ledBuffer.setRGB(i, r2, g2, b2);
        }
      }
    }else if(ledMode == FancyLED.KNIGHT_RIDER){
      for(int i = 0; i < ledBuffer.getLength(); ++i){
        ledBuffer.setRGB(i, r2, g2,b2);
      }
      if(lastI == 140){
        lastI = 0;
      }
      ++lastI;
      for(int i = lastI; i < lastI+40; ++i){
        ledBuffer.setRGB(i, r,g,b);
      }
    }else if(ledMode == FancyLED.PULSE){
      for(int i = 0; i < ledBuffer.getLength(); ++i){
        ledBuffer.setRGB(i, (pulseRun <= pulseRange ? r : r2) , (pulseRun <= pulseRange ? g : g2), (pulseRun <= pulseRange ? b : b2));
      }

      ++pulseRun;
      if(pulseRun == pulseMax){
        pulseRun = 0;
      }
    }
    LED.setData(ledBuffer);
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
    LED.setData(ledBuffer);
  }

  public static enum FancyLED{
      ONE_ON_ONE,
      KNIGHT_RIDER,
      PULSE;
  }
}
