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
  /** Creates a new LimeLight. */
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  double x;
  double y;
  double goalHeightInches = 20.0;
  double targetDistance;
  boolean inRange;

  public LimeLight() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");

    // read values periodically
    

    // post to smart dashboard periodically

  }

  @Override
  public void periodic() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?

    // distance from the center of the Limelight lens to the floor

    // distance from the target to the floor

    double ATGR = Math.toRadians(goalHeightInches);



    //calculate distance
    targetDistance = (goalHeightInches)/Math.tan(ATGR);

    if((targetDistance <= 40 && targetDistance >= 35) || (targetDistance <= 33 && targetDistance >= 30))
    {
      inRange = true;
      if(x < 25 && x > 7)
    
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("target_distance", targetDistance);

  }
}
}