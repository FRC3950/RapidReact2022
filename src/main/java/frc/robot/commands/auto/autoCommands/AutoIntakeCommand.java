// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.autoCommands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.teleop.IntakeCommand;
import frc.robot.misc.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoIntakeCommand extends CommandBase {
  /** Creates a new AutoIntakeCommand. */
  IntakeSubsystem intake;
  ShooterSubsystem shooter;
  boolean finished = false;

  public AutoIntakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {

    this.intake = intake;
    this.shooter = shooter; 

    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setSolenoid(Constants.States.DOWN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.getSensorValues()[0] && shooter.getSensorValues()[1]){
      intake.intake(0.75);
      shooter.setConveyor(0.6);
      shooter.setIndexer(0.5);
    }
    else if(!shooter.getSensorValues()[0]){
      shooter.setConveyor(0.0);
      intake.intake(0.0);
      shooter.setIndexer(0.0);
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSolenoid(Constants.States.UP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
