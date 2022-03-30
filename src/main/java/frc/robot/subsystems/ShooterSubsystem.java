// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.revrobotics.ColorMatch;
//import com.revrobotics.ColorMatchResult;
//import com.revrobotics.ColorSensorV3;
//import com.revrobotics.ColorSensorV3.*;


import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   *
   */
  /** Creates a new ShooterSubsystem. */

  private final WPI_TalonFX bottom = new WPI_TalonFX(Constants.bottom);
  private final WPI_TalonFX top = new WPI_TalonFX(Constants.top);
  private final WPI_TalonSRX conveyor = new WPI_TalonSRX(Constants.conveyor);
  private final WPI_TalonSRX indexer = new WPI_TalonSRX(Constants.indexer);

  private DigitalInput[] irBeams = new DigitalInput[] {
    new DigitalInput(3),
    new DigitalInput(5)
  };

  private static final double kP_vel1 = 0.010005, kP_vel2 = 0.010005;
  private static final double kI_vel1 = 0.0000009536743, kI_vel2 = 0.0000009536743;
  private static final double kF= 0.0451;
  private static final double kD_vel1 = 0.000006, kD_vel2 = 0.000006;
  private static final double closed_loop_ramp = 0.2;

  public int targetTopVelocity = 9990; 
  public int targetBottomVelocity = 11027; 

  public ShooterSubsystem() {

    conveyor.setNeutralMode(NeutralMode.Brake);
    indexer.setNeutralMode(NeutralMode.Brake);

    bottom.configFactoryDefault();
    top.configFactoryDefault();

    bottom.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10); //Read more into timeout Param
    top.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

    bottom.setInverted(false);
    top.setInverted(true);
    bottom.setSensorPhase(false);
    top.setSensorPhase(false);
    
    bottom.config_kF(0, kF);
    top.config_kF(0, kF);

    bottom.config_kP(0, kP_vel1);
    top.config_kP(0, kP_vel2);

    bottom.config_kI(0, kI_vel1);
    top.config_kI(0, kI_vel2);

    bottom.config_kD(0, kD_vel1);
    top.config_kD(0, kD_vel2);
  }

  public void motorOn(){
    bottom.set(ControlMode.Velocity, 11027);
    top.set(ControlMode.Velocity, 9990);
  }

  /** @param b : Bottom motor velocity (Default: 11027)
   *  @param t : Top motor velocity (Default: 9990) */
  
  public void motorOn(double b, double t){
    bottom.set(ControlMode.Velocity, b);
    top.set(ControlMode.Velocity, t);  
  }

  public double[] getCurrentVelocities(){
    return new double[] {
      bottom.getSelectedSensorVelocity(0),
      top.getSelectedSensorVelocity(0)
    };
  }

  // public void decrementTargetVelocity(){
  //   targetTopVelocity -= 200;
  //   targetBottomVelocity -= 200;
  // }

  // public void incrementTargetVelocity(){
  //   targetTopVelocity += 200;
  //   targetBottomVelocity += 200;
  // }

  // public void resetShoot(){
  //   targetTopVelocity = 11000;
  //   targetBottomVelocity = 11500;
  // }

  //Conveyor stuff:
  public void setConveyor(final double speed){
    conveyor.set(speed);
  }

  public void setIndexer(final double speed){
    indexer.set(speed);
  }

  public void outtake(double speed){
    speed = -Math.abs(speed);
    indexer.set(speed);
    conveyor.set(speed);
  }

  /** @return conveyor sensor [0] value and indexer sensor [1] value */
 public boolean[] getSensorValues(){
   return new boolean[] {
      irBeams[0].get(),
      irBeams[1].get()
    };
  }
  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Indexer", irBeams[0].get());
    SmartDashboard.putBoolean("Conveyor", irBeams[1].get());

    SmartDashboard.putNumber("Bottom shooter actual speed:", getCurrentVelocities()[0]);
    SmartDashboard.putNumber("Top shooter actual speed:", getCurrentVelocities()[1]);

  }
}
