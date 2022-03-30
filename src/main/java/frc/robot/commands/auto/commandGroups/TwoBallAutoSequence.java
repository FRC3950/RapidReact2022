// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.commandGroups;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.commands.auto.autoCommands.*;
import frc.robot.commands.auto.commandGroups.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAutoSequence extends SequentialCommandGroup {
  /** Creates a new DriveSequence. */
  public TwoBallAutoSequence(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShootCommand(11027, 9990, 2.0, shooter),
      new IntakeDrive(40000, drivetrain, shooter, intake),
      new AutoShootCommand(12557.83, 15327.57, 4.0, shooter)
    );
  }
}
