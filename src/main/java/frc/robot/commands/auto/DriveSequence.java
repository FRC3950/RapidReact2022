// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveSequence extends SequentialCommandGroup {
  /** Creates a new DriveSequence. */
  public DriveSequence(DrivetrainSubsystem drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(()->drivetrain.encoderDrive(0.5, 20000), drivetrain),
      new RunCommand(()->drivetrain.encoderDrive(-0.5, 20000), drivetrain),
      new RunCommand(()->drivetrain.timeDrive(0.5, 20000), drivetrain)
    );
  }
}
