// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.fasterxml.jackson.databind.cfg.ConfigFeature;

public class LedSubsystem extends SubsystemBase {
  private static final CANdle led = new CANdle(17, "rio");
  private static CANdleConfiguration configAll = new CANdleConfiguration();

  private static int channel = 0;
  private static final int ledCount = 139;
  /** Creates a new ledSubsystem. */

  public static enum State{
    OFF,
    ON
  }

  private static final SendableChooser<State> ledState = new SendableChooser<>();

  public LedSubsystem() {
    configAll.stripType = LEDStripType.GRB;
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.brightnessScalar = 0.50;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;

    ledState.addOption("LEDS OFF", State.OFF);
    ledState.addOption("LEDS ON", State.ON);

    SmartDashboard.putData("Leds on/off", ledState);
  }

  public static void setColor(int r, int g, int b){
    led.setLEDs(r, g, b);
  }

  public static void setStrobe(int r, int g, int b){
    led.animate(
      new StrobeAnimation(r, g, b)
    );
  }

  public State getSelected(){
    return ledState.getSelected();
  }

  //Cyan: 27, 239, 239
  @Override
  public void periodic() {
    if(getSelected() == State.ON && getSelected() != null){
      if(!ShooterSubsystem.getSensorValues()[0] && !ShooterSubsystem.getSensorValues()[1]){
        setColor(0, 255, 0);
      }
      else if(!ShooterSubsystem.getSensorValues()[0] || !ShooterSubsystem.getSensorValues()[1]){
        setColor(255, 0, 0);
      }
      else {
        led.animate(
          new LarsonAnimation(28, 228, 178, 22, 0.9, 139, BounceMode.Front, 7)       // new RainbowAnimation(0.35, 0.3, ledCount)
        );
        // led.animate(new FireAnimation(0.5, 0.5, ledCount, 0.5, 0.2));
      }
    }
    else {
      led.setLEDs(0, 0, 0);
    } 
  }
}
