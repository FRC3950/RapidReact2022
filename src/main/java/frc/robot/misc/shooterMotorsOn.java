// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterMotorsOn extends CommandBase {
  /** Creates a new shooterMotorsOn. */
  private double bottom, top, conveyor;
  private final ShooterSubsystem shooterSubsystem;
  
  public ShooterMotorsOn(ShooterSubsystem shooterSubsystem, double conveyor, double top, double bottom) {
    this.shooterSubsystem = shooterSubsystem;
    this.top = top;
    this.bottom = bottom;
    this.conveyor = conveyor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.motorOn();
    SmartDashboard.putNumber("bottom U/100MS (Default 11,000)", bottom);
    SmartDashboard.putNumber("top U/100MS (Default 10,500)", top);
    SmartDashboard.putNumber("Conveyor Output", conveyor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.motorOn(
      SmartDashboard.getNumber("bottom U/100MS (Default 11,000)", bottom),
      SmartDashboard.getNumber("top U/100MS (Default 10,500)", top),
      SmartDashboard.getNumber("Conveyor Output", conveyor)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.motorOn(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
