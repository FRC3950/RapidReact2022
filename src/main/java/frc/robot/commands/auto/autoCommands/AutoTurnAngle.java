// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoTurnAngle extends CommandBase {
  /** Creates a new AutoTurnAngle. */
  private double targetAngle, speed;
  private final DrivetrainSubsystem drivetrain;
  public AutoTurnAngle(double targetAngle, double speed, DrivetrainSubsystem drivetrain) {
    this.targetAngle = targetAngle;
    this.speed = speed;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.teleDrive(0, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (drivetrain.getAngle() >= targetAngle); 
  }
}
