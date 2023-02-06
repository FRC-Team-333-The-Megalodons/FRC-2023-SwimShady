// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.TurnDegrees;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.NavX;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraight extends SequentialCommandGroup {
  /** Creates a new DriveStraight. */
  public DriveStraight(Chassis chassis, NavX navX) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Drive(chassis, navX, false,30),
      new Drive(chassis, navX, true,100),
      new WaitCommand(.8),
      new TurnDegrees(chassis, navX, 180),
      new WaitCommand(1),
      new Drive(chassis, navX, true,80),
      new WaitCommand(.8),
      new TurnDegrees(chassis, navX, 180)
    );
  }
}
