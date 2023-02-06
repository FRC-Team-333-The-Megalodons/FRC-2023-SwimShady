// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {
  /** Creates a new NavX. */

  AHRS navx;

  public NavX() {
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

  public double getTilt(){
    //todo get rid of gyro noise
    return navx.getPitch();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("tilt", (getTilt()-navx.getRawAccelX()) > 0 ? (getTilt()-navx.getRawAccelX())-2 : (getTilt()-navx.getRawAccelX())+2);
    SmartDashboard.putNumber("Z", navx.getRawGyroZ());
    SmartDashboard.putNumber("accel", navx.getCompassHeading());
    SmartDashboard.putNumber("uhhh2", navx.getAngle());
  }
}
