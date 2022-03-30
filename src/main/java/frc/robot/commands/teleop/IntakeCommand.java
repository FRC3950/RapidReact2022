// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;



import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.misc.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  IntakeSubsystem intake; 
  ShooterSubsystem shooter;
  boolean ballLoaded = true;

  public IntakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    this.intake = intake;
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!(!shooter.getSensorValues()[0] && !shooter.getSensorValues()[1])){
      intake.setSolenoid(Constants.States.DOWN);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //No balls
    if(shooter.getSensorValues()[0] && shooter.getSensorValues()[1]){
      intake.intake(0.75);
      shooter.setConveyor(0.6);
      shooter.setIndexer(0.40);
    }
    else if(!shooter.getSensorValues()[1] && shooter.getSensorValues()[0]){
      shooter.setConveyor(0.6);
      intake.intake(0.75);
    }
    //Ball indexer nothing below
    else if(!shooter.getSensorValues()[0] && shooter.getSensorValues()[1]){
      intake.intake(0.75);
      shooter.setConveyor(0.55);
      shooter.setIndexer(0.0);
    }
    //Full
    else if(!shooter.getSensorValues()[0] && !shooter.getSensorValues()[1]){
      intake.intake(0.0);
      shooter.setConveyor(0.0);
      shooter.setIndexer(0.0);
    }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSolenoid(Constants.States.UP);
    intake.intake(0);
    shooter.setConveyor(0);
    shooter.setIndexer(0);
  }
  // Josh was made in Korea! 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
