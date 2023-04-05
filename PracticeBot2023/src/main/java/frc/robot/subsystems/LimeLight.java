// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

public class LimeLight extends SubsystemBase {
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  double x;
  double y;
  boolean inRange;
  double targetHeight = 20;// Change when nessesary
  double targetDistance;
  double angletoRad;

  /** Creates a new LimeLight. */
  public LimeLight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");

  }

  @Override
  public void periodic() {

    x = tx.getDouble(0); // Max value : 24
    y = ty.getDouble(0); // Max value : 27
    // Get's x and y values

    angletoRad = Math.toRadians(y);

    targetDistance = (targetDistance / Math.tan(angletoRad));

    if ((targetDistance > 31)) {
      inRange = false;
    } else {
      if(x > 10 && x < 24) // ADJUST THESE VALUES IN ORDER TO INCREASE ACCURANCY FOR ALIGNMENT. REQUIRES AT LEAST A 7 DEGREE TOLERANCE AREA.
      {
        inRange = false;
      }
      inRange = false;
    }

    

    SmartDashboard.putNumber("LimelightXangle", x);
    SmartDashboard.putNumber("LimelightYangle", y);
    SmartDashboard.putNumber("LimeLightDistance", targetDistance);//distance in inches
    SmartDashboard.putBoolean("Is_In_Range", inRange);

  }
}
