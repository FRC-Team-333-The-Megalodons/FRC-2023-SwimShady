// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.NavX;

public class TurnDegrees extends CommandBase {

  Chassis chassis;
  NavX gyro;
  double degrees;

  private final double kP = 0.005;
  private final double kI = 0.00026;
  private final double kD = 0;
  private final double iLim = 62;
  private final double maxTolerance;
  private final double minTolerance;

  private double error = 0;
  private double output = 0;
  private double errorSum = 0;
  private double lastTimeStamp = 0;
  double currentTime = 0;

  boolean isGyroNearTarget = false;

  /** Creates a new TurnDegrees. */
  public TurnDegrees(Chassis chassis, NavX gyro, double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    addRequirements(gyro);
    this.chassis = chassis;
    this.degrees = degrees;
    this.gyro = gyro;

    this.maxTolerance = degrees + .3;
    this.minTolerance = degrees - .3;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro.reset();
  }

  boolean greaterThanMax = false;
  boolean lowerThanMin = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp() - lastTimeStamp;
    
    if(gyro.getAngle() > maxTolerance){
      error = maxTolerance - gyro.getAngle();
      greaterThanMax = true;
      lowerThanMin = false;
    }

    if(gyro.getAngle() < minTolerance){
      error = minTolerance - gyro.getAngle();
      greaterThanMax = false;
      lowerThanMin = true;
    }

    SmartDashboard.putBoolean("Greater than max", greaterThanMax);
    SmartDashboard.putBoolean("lower than min", lowerThanMin);

    if(Math.abs(error) < iLim && Math.round(error) != 0){
      errorSum += error + currentTime;
    }

    output = (kP * error) + (kI * errorSum);
    chassis.drive(-output, 0);
    
    lastTimeStamp = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("Sum", errorSum);
    isGyroNearTarget = (maxTolerance) >= gyro.getAngle() && (minTolerance) <= gyro.getAngle();
    SmartDashboard.putBoolean("Is near", isGyroNearTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isGyroNearTarget;
  }
}
