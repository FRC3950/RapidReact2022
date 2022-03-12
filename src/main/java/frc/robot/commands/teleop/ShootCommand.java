// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {
  /** Creates a new ShootCommand. */
  ShooterSubsystem shooter;
  private final Timer timer = new Timer();
  
  double targetspeedB, targetspeedT;
  double currentspeedB, currentSpeedT;

  
  public ShootCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);

    SmartDashboard.putNumber("Top shooter speed:", shooter.getTargetVelocities()[1]);
    SmartDashboard.putNumber("Bottom shooter speed:", shooter.getTargetVelocities()[0]);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetspeedB = Math.abs(shooter.getTargetVelocities()[0]);
    targetspeedT = Math.abs(shooter.getTargetVelocities()[1]);

    currentspeedB = Math.abs(shooter.getCurrentVelocities()[0]);
    currentSpeedT = Math.abs(shooter.getCurrentVelocities()[1]);
    
  
    if(currentspeedB >= targetspeedB - 200 && currentspeedB <= targetspeedB + 200 
    && currentSpeedT >= targetspeedT - 200 && currentSpeedT <= targetspeedT + 200){
      System.out.println(targetspeedB);
      System.out.println(currentspeedB);
      shooter.motorOn(-targetspeedB, -targetspeedT, 0.95);
    }
    else{
      shooter.motorOn(-targetspeedB, -targetspeedT, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.motorOn(0, 0, 0);
    IntakeSubsystem.ballCount = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
