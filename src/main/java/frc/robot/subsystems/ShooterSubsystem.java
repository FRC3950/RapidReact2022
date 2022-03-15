// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private final WPI_TalonFX bottom = new WPI_TalonFX(Constants.bottom);
  private final WPI_TalonFX top = new WPI_TalonFX(Constants.top);
  private final WPI_TalonSRX conveyor = new WPI_TalonSRX(Constants.conveyor);

  //private final DigitalInput sensor = new DigitalInput(Constants.sensor);
  private final DigitalInput intakeSensor = new DigitalInput(0); 

  private static final double kP_vel1 = 0.010005, kP_vel2 = 0.010005;
  private static final double kI_vel1 = 0.0000009536743, kI_vel2 = 0.0000009536743;
  private static final double kF= 0.0451;
  private static final double kD_vel1 = 0.000006, kD_vel2 = 0.000006;
  private static final double closed_loop_ramp = 0.2;
  //public static final int internal_zone = 100; //likely not needed

  public int targetTopVelocity = 10900;
  public int targetBottomVelocity = 11400;

  private int decrements = 0;
  private int increments = 0;

  public ShooterSubsystem() {

    bottom.configFactoryDefault();
    top.configFactoryDefault();

    bottom.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0, 10); //Read more into timeout Param
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

    // bottom.configClosedloopRamp(closed_loop_ramp);
    // top.configClosedloopRamp(closed_loop_ramp);

  }

  public void motorOn(){
    bottom.set(ControlMode.Velocity, 11000);
    top.set(ControlMode.Velocity, 10500);
  }

  /** @param b : Bottom motor velocity (Default: 11000)
   *  @param t : Top motor velocity (Default: 10500)
   *  @param c : Conveyor speed (< 1.0) */
  
  public void motorOn(double b, double t){
    bottom.set(ControlMode.Velocity, b);
    top.set(ControlMode.Velocity, t);  
  }

  /** @return Returns double[] of target velocities (set w/ SmartDashboard) of bottom motor [0] and top motor [1]*/
  public double[] getTargetVelocities(){
    return new double[]{
      targetBottomVelocity, targetTopVelocity
    };
  }

  public double[] getCurrentVelocities(){
    return new double[] {
      bottom.getSelectedSensorVelocity(0),
      top.getSelectedSensorVelocity(0)
    };
  }

  public void decrementTargetVelocity(){
    targetTopVelocity -= 200;
    targetBottomVelocity -= 200;
  }

  public void incrementTargetVelocity(){
    targetTopVelocity += 200;
    targetBottomVelocity += 200;
  }

  //Conveyor stuff:
  public void setConveyor(final double speed){
    conveyor.set(speed);
  }

  public void outtake(double speed){
    speed = -Math.abs(speed);

    conveyor.set(speed);
    bottom.set(speed);
    top.set(speed);
  }

  public boolean getIntakeSensor(){
    return intakeSensor.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Bottom target velocity value", targetBottomVelocity);
    SmartDashboard.putNumber("Top target velocity value", targetTopVelocity);

    SmartDashboard.putNumber("Bottom shooter speed:", getCurrentVelocities()[0]);
    SmartDashboard.putNumber("Top shooter speed:", getCurrentVelocities()[1]);

    
  }
}
