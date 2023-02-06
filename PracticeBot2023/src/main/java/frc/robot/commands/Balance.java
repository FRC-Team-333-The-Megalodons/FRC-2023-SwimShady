// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.NavX;

public class Balance extends CommandBase {
  /** Creates a new Balance. */

  private final Chassis drivetrain;
  private final NavX gyro;

  private final double kP = 0.008;
  private final double kI = 0.0001;
  private final double kD = 0;
  private final double iLim = 30;
  private final double maxTolerance = 1;
  private final double minTolerance = -1;

  private double error = 0;
  private double output = 0;
  private double errorSum = 0;
  private double lastTimeStamp = 0;
  double currentTime = 0;

  public Balance(Chassis chassis, NavX navX) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    addRequirements(navX);
    this.drivetrain = chassis;
    this.gyro = navX;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  boolean greaterThanMax = false;
  boolean lowerThanMin = false;
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
    drivetrain.drive(0, -output);
    
    lastTimeStamp = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("Sum", errorSum);
    //this may or may not work lol   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
