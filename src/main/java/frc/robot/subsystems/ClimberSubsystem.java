// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  WPI_TalonFX master;
  WPI_TalonFX follower;

  DoubleSolenoid winch, pivot;

  private final DigitalInput rightLimitSwitch = new DigitalInput(1);
  private final DigitalInput leftLimitSwitch = new DigitalInput(2);

  public ClimberSubsystem() {
    master = new WPI_TalonFX(Constants.master);
    follower = new WPI_TalonFX(Constants.follower);

    winch = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, Constants.winch[0], Constants.winch[1]);
    pivot = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, Constants.pivot[0], Constants.pivot[1]);

    follower.follow(master);

    master.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    follower.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    master.setInverted(true);
    follower.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("CLIMBER LOCK", getLock());
    SmartDashboard.putString("Climber solenoid", winch.get().toString());
    SmartDashboard.putNumber("Climber encoder count", getEncoderCounts()[0]);
    
  }

  public void setMotors(double speed){
    master.set(speed);
  }

  public void setSolenoid(Value value){
    winch.set(value);
  }

  public void toggleSolenoid(){
    if(winch.get() == Value.kReverse){
      winch.set(Value.kForward);
    }
    else if(winch.get() != Value.kReverse){
      winch.set(Value.kReverse);
    }
  }

  public void togglePivot(){
    if(pivot.get() == Value.kReverse){
      pivot.set(Value.kForward);
    } 
    else if(pivot.get() != Value.kReverse){
      pivot.set(Value.kReverse);
    }
  }

  public double[] getEncoderCounts(){
    return new double[]{
      master.getSelectedSensorPosition(),
      follower.getSelectedSensorPosition()
    };
  }

  public boolean[] getSensorValues(){
    return new boolean[] {
      rightLimitSwitch.get(),
      leftLimitSwitch.get()
    };
  }

  public boolean getLock(){
    return (winch.get() == Value.kForward);
  }

  public void resetEncoderCount(){
    master.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

}
