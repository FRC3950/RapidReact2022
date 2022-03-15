// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends CommandBase {
  /** Creates a new ClimberCommand. */
  private final ClimberSubsystem climber;
  private final DoubleSupplier y;

  private final DigitalInput rightLimitSwitch = new DigitalInput(1);
  private final DigitalInput leftLimitSwitch = new DigitalInput(2);

  private boolean islimitRight, islimitLeft;

  public ClimberCommand(DoubleSupplier y, ClimberSubsystem climber) {
    this.climber = climber;
    this.y = y;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setSolenoid(Value.kReverse);
    climber.resetEncoderCount();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      if(y.getAsDouble() > 0.1/**add encoder later */){
        climber.setSolenoid(Value.kReverse);
        climber.setMotors(y.getAsDouble());
      }
      else if(y.getAsDouble() < -0.1 && rightLimitSwitch.get() && leftLimitSwitch.get()){
        climber.setSolenoid(Value.kReverse);
        climber.setMotors(y.getAsDouble());
      }
      else{
        climber.setSolenoid(Value.kForward);
        climber.setMotors(0.0);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setSolenoid(Value.kForward);
    climber.setMotors(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
