// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.ChargeStation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.roboAutos.oneOnly.ScoreHighCube;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Gyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighCubePlusEngage extends SequentialCommandGroup {
  /** Creates a new ScoreHighCubePlusEngage. */
  public ScoreHighCubePlusEngage(frc.robot.subsystems.Elevator elevator, frc.robot.subsystems.Intake intake, Chassis chassis, Gyro gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreHighCube(elevator, intake)
      ,new DockAndEngage(chassis, gyro)
    );
  }
}
