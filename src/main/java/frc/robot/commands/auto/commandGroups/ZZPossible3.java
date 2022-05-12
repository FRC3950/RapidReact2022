// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.autoCommands.AutoEncoderDrive;
import frc.robot.commands.auto.autoCommands.AutoShootCommand;
import frc.robot.commands.teleop.IntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZZPossible3 extends SequentialCommandGroup {
  /** Creates a new ThreeBallAutoExpMkII. */
  public ZZPossible3(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
   
    addCommands(
      new AutoShootCommand(11027, 9990, 2.0, shooter, intake),
      new ParallelRaceGroup(new AutoEncoderDrive(40000, 0.6, drivetrain), new IntakeCommand(intake, shooter)),
      new AutoShootCommand(12557.83, 15327.57, 4.0, shooter, intake)
    );
  }
}
