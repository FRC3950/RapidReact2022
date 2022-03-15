// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  IntakeSubsystem intake; 
  ShooterSubsystem shooter;
  boolean ballLoaded = false;
  public IntakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    this.intake = intake;
    this.shooter = shooter;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setSolenoid(Value.kReverse);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intake(0.7);
    
    //shooter.setConveyor(0.0);
    // if(shooter.getIntakeSensor() == false){
    //   shooter.setConveyor(0.3);
     
    // }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSolenoid(Value.kForward); //inverted
    intake.intake(0);
    shooter.setConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
