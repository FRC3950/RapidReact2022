// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutoShootCommand extends CommandBase {
  /** Creates a new AutoShootCommand. */
  ShooterSubsystem shooter;
  private final Timer timer = new Timer();

  double time;
  double currentTime = 0;

  double targetspeedB, targetspeedT;
  double currentspeedB, currentSpeedT;

  public AutoShootCommand(double targetspeedB, double targetspeedT, final double time, ShooterSubsystem shooter) {
    this.shooter = shooter;
    this.time = time; 
    this.targetspeedB = targetspeedB;
    this.targetspeedT = targetspeedT;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = timer.get();

    currentspeedB = Math.abs(shooter.getCurrentVelocities()[0]);
    currentSpeedT = Math.abs(shooter.getCurrentVelocities()[1]);
    

    shooter.motorOn(-targetspeedB, -targetspeedT);
    // shooter.setConveyor(0.5);
    // shooter.setIndexer(0.5);
  
    if(currentspeedB >= targetspeedB - 550 && currentspeedB <= targetspeedB + 550 
    && currentSpeedT >= targetspeedT - 550 && currentSpeedT <= targetspeedT + 550){
      
      if(shooter.getSensorValues()[0] == true){
        shooter.setConveyor(0.7);
      }
      shooter.setIndexer(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.motorOn(0,0);
    shooter.setConveyor(0.0);
    shooter.setIndexer(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentTime >= time);
  }
}
