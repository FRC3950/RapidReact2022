// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.autoCommands.AutoEncoderDrive;
import frc.robot.commands.auto.autoCommands.AutoShootCommand;
import frc.robot.commands.auto.autoCommands.AutoTurn;
import frc.robot.commands.auto.autoCommands.StartTimer;
import frc.robot.commands.teleop.LimeShot;
import frc.robot.commands.teleop.LimelightCenterCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallExperimental_HumanPlayerSide extends SequentialCommandGroup {
  /** Creates a new ThreeBallExperimental_HumanPlayerSide. */
  public ThreeBallExperimental_HumanPlayerSide(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake, LimelightSubsystem lime) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(     
    new AutoShootCommand(11027, 9990, 2.0, shooter),
    new IntakeDrive(40000, drivetrain, shooter, intake),
    new AutoShootCommand(12557.83, 15327.57, 4.0, shooter),
    new AutoTurn(-122.25, drivetrain),
    new IntakeDrive(40000, drivetrain, shooter, intake),
    new AutoEncoderDrive(-20000, .6, drivetrain),
    new AutoTurn(90, drivetrain),
    new ParallelRaceGroup(new StartTimer(2),new LimelightCenterCommand(lime, drivetrain)),
    new LimeShot(lime, shooter, intake)
    );
  }
}
