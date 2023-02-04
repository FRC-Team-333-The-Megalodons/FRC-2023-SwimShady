// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.NavX;

public class DriveForwards extends CommandBase {

  private final Chassis drivetrain;
  private final NavX gyro;

  private final double DRIVE_SPEED = 0.5;

  private final double kP = 0.05;
  private final double kI = 0.001;
  private final double kD = 0;
  private final double iLim = 18;
  private final double maxTolerance = .1;
  private final double minTolerance = -.1;

  private double error = 0;
  private double output = 0;
  private double errorSum = 0;
  private double lastTimeStamp = 0;

  double currentTime = 0;
  double autoTimeMils = 5000;
  double timeLapse = 0;
  double timeStamp = 0;

  /** Creates a new DriveForwards. */
  public DriveForwards(Chassis chassis, NavX gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    addRequirements(gyro);
    this.drivetrain = chassis;
    this.gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro.reset();
    timeStamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  boolean greaterThanMax = false;
  boolean lowerThanMin = false;
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp() - lastTimeStamp;
    timeLapse += Timer.getFPGATimestamp() - timeStamp;
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
    drivetrain.drive(-output, .6);
    
    lastTimeStamp = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("Sum", errorSum);
    SmartDashboard.putNumber("time",timeLapse);
    //this may or may not work lol   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeLapse  >= 300;
  }
}