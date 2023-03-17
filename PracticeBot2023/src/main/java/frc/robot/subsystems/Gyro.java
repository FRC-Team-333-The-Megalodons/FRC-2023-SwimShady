// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  /** Creates a new Gyro. */
   AHRS navx;
   DecimalFormat df1 = new DecimalFormat("0.#");

   public double getUsableTilt(){
    return Double.valueOf(df1.format(getTilt()))-4;
   }

   public Gyro() {
     navx = new AHRS();
     navx.reset();
     navx.calibrate();
   }
 
   public void reset(){
     navx.reset();
   }
 
   public double getAngle(){
     return navx.getAngle();
   }
 
   private double getTilt(){
     //todo get rid of gyro noise
     return navx.getPitch();
   }
 
   @Override
   public void periodic() {
     // This method will be called once per scheduler run
     SmartDashboard.putNumber("tilt", getUsableTilt());
     SmartDashboard.putNumber("Z", navx.getRawGyroZ());
     SmartDashboard.putNumber("accel", navx.getCompassHeading());
     SmartDashboard.putNumber("x", navx.getAngle());
   }
}
