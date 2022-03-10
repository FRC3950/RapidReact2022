// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class CenterCommand extends CommandBase {
  /** Creates a new CenterCommand. */
  private final DrivetrainSubsystem drivetrain;
  private boolean withinRange = false;
  public CenterCommand(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(drivetrain.getAngle() > 5.0){
      drivetrain.turn(0.25);
    } 
    else if(drivetrain.getAngle() < -5.0){
      drivetrain.turn(-0.25);
    }
    else {
      withinRange = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.turn(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return withinRange;
  }
}
