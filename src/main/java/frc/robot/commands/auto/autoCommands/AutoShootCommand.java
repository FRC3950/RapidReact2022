// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutoShootCommand extends CommandBase {
  /** Creates a new AutoShootCommand. */
  ShooterSubsystem shooter;
  Timer timer = new Timer();

  double time;
  double currentTime = 0;

  public AutoShootCommand(ShooterSubsystem shooter, final double time) {
    this.shooter = shooter;
    this.time = time + 2; //period for shooters to spin up
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    timer.start();
    currentTime = timer.get();
    
    if(timer.get() <= 2){
      shooter.motorOn(12500, 12000, 0.0);
    }
    else {
      shooter.motorOn(12500, 12000, 1.00);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.motorOn(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentTime >= time);
  }
}
