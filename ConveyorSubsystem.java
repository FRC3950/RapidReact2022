// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.prototypes;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;

public class ConveyorSubsystem extends SubsystemBase {
  /** Creates a new ConveyorSubsystem. */

  private final WPI_TalonSRX motor = new WPI_TalonSRX(5);
  private final DigitalInput sensor = new DigitalInput(1);

  public ConveyorSubsystem() {
    //PID stuff with motor will go here
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Conveyor Sensor", getSensor());
  }

  public void setMotor(final double speed){
      motor.set(speed);
  }

  public void outtake(double speed){
    if(speed > 0.0) speed *= -1;
    motor.set(speed);
  }

  public boolean getSensor(){
    return sensor.get();
  }
  
}
