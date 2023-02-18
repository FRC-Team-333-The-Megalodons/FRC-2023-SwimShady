// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */
  CANSparkMax rightmotor1, rightmotor2, rightmotor3;
  CANSparkMax leftmotor1, leftmotor2, leftmotor3;
  MotorControllerGroup rightleader;
  MotorControllerGroup leftleader;
  Joystick stick;
  DifferentialDrive drive;

  ColorSensorV3 cs;

  public Chassis() {
    rightmotor1  = new CANSparkMax(1, MotorType.kBrushless);
    rightmotor2  = new CANSparkMax(2, MotorType.kBrushless);
    rightmotor3  = new CANSparkMax(3, MotorType.kBrushless);

    rightleader = new MotorControllerGroup(rightmotor1, rightmotor2, rightmotor3);

    leftmotor1 = new CANSparkMax(4, MotorType.kBrushless);
    leftmotor2 = new CANSparkMax(5, MotorType.kBrushless);
    leftmotor3 = new CANSparkMax(6, MotorType.kBrushless);

    leftleader = new MotorControllerGroup(leftmotor1, leftmotor2, leftmotor3);

    rightleader.setInverted(true);

    stick = new Joystick(0);
    drive = new DifferentialDrive(leftleader, rightleader);

    cs = new ColorSensorV3(I2C.Port.kOnboard);
  
    SmartDashboard.getNumber("Blue", cs.getBlue());
    SmartDashboard.getNumber("Red", cs.getRed());
    SmartDashboard.getString("Color", cs.getColor().toString());
  }

    public boolean isCone(){
    boolean isCone = false;
    if(cs.getBlue()==4444 && cs.getRed()==7777 || cs.getColor().toString()=="#5F900F"){
        isCone = true;
        System.out.println("workin");
    } else{
      isCone = false;
    }
    return isCone;
  }
  }


    //TODO: Amina, add your logic here!!
   // boolean isCone = false;

   /*  if( 
      cs.getColor().toString()=="#5F900F") {
      isCone = true;
      System.out.println("oui")
    } else {
     isCone = false;
    }

   return isCone;
  }

  public boolean isCube() {

    //TODO: Amina, add your logic here!!

    boolean isCube;   
   if( cs.getColor().toString()=="#33507A") {
      isCube = true;
    } else {
      isCube = false;
    }
    return isCone();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drive.arcadeDrive(-stick.getX(), -stick.getY());

    SmartDashboard.putBoolean("Is Cone?", isCone());
    SmartDashboard.putString("Color", cs.getColor().toString());
  }
  */



  
