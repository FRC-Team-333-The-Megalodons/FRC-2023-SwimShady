// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.ChargeStation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.roboActions.Combo.GoHome;
import frc.robot.commands.roboAutos.ChargeStation.stages.DockAndEngage;
import frc.robot.commands.roboAutos.oneOnly.ScoreMidCone;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IntakeAlternate;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreMidConePlusEngage extends SequentialCommandGroup {
  /** Creates a new ScoreHighPlusEngage. */
  public ScoreMidConePlusEngage(Elevator elevator, IntakeAlternate intake, Chassis chassis, Gyro gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreMidCone(elevator, intake,chassis,gyro)
      ,new GoHome(intake, elevator)
      ,new DockAndEngage(chassis, gyro)
    );
  }
}
