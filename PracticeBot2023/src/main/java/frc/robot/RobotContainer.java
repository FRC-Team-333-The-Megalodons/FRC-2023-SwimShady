// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveForwards;
import frc.robot.commands.autos.DriveStraight;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.NavX;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // ++The robot's subsystems and commands are defined here...

  private final Chassis chassis = new Chassis();
  private final NavX navX = new NavX();

  //private final DriveForwards driveForwardsCommand = new DriveForwards(chassis);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  }

  public void periodic() {
    chassis.periodic();
    //navX.periodic();
  }

  public void displayVals(){
    navX.periodic();
  }

  public void reset(){
    navX.reset();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new DriveStraight(chassis, navX);
  }
}
