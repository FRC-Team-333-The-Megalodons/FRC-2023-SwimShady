// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.ChargeStation.stages;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.roboActions.drive.Turn;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Gyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DockAndEngage extends SequentialCommandGroup {
  /** Creates a new DockAndEngage. */
  frc.robot.utils.PIDController turnController;
  public DockAndEngage(Chassis chassis, Gyro gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    turnController = new frc.robot.utils.PIDController(.0045, .0038, 0, 60, .35, .35, 0);
    addCommands(
      new Turn(chassis, gyro, 180, turnController, true)
      ,new Unbalance(chassis, gyro)
      ,new Balance(chassis, gyro)
    );
  }
} 
