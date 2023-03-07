// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team6560.frc2023.Constants;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  CANSparkMax m_wrist;
  CANSparkMax m_claw;
  DutyCycleEncoder wristEncoder;

  double curAngle = 0.0;
  int whatButtonPressed = 0;
  boolean wedonotwantogoupanymore = false;
  boolean wedonotwantogodownanymore = false;

  double upperLimit  = Constants.WristConstants.upperLimit;
  double lowerLimit = Constants.WristConstants.lowerLimit;
  boolean disableLimits = true;

  
  /** Creates a new Wrist. */
  public Wrist() {
    m_wrist = new CANSparkMax(Constants.WristConstants.WristID, MotorType.kBrushless);
    m_claw = new CANSparkMax(Constants.WristConstants.ClawID, MotorType.kBrushless);

    m_wrist.setInverted(false);
    m_claw.setInverted(false);

    m_wrist.setIdleMode(IdleMode.kBrake);
    m_claw.setIdleMode(IdleMode.kBrake);

    wristEncoder = new DutyCycleEncoder(Constants.WristConstants.WristEncoderID);
    wristEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

    SmartDashboard.putNumber("Current Wrist Angle", curAngle);
    SmartDashboard.putNumber("Wrist Upper Limit", upperLimit);
    SmartDashboard.putNumber("Wrist Lower Limit", lowerLimit);
    SmartDashboard.putBoolean("Disable Wrist Limits", disableLimits);
  }

  @Override
  public void periodic() {
    curAngle = wristEncoder.getAbsolutePosition() * 360;

    upperLimit = SmartDashboard.getNumber("Wrist Upper Limit", upperLimit);
    lowerLimit = SmartDashboard.getNumber("Wrist Lower Limit", lowerLimit);
    disableLimits = SmartDashboard.getBoolean("Disable Wrist Limits", disableLimits);
    // This method will be called once per scheduler run
  }

  public boolean getDisableLimitsBoolean() {
    return disableLimits;
  }

  public void setDisableLimitsBoolean(boolean userInput) {
    disableLimits = userInput;
  }

  public void buttonPressed(int whichButton) {
    whatButtonPressed = whichButton;
  }

  public void setWristSpeed(double wristSpeed) {
    if( wristSpeed > 0.2 ) {
      if(wedonotwantogodownanymore) {
        if( whatButtonPressed == 1 ) {
          m_wrist.set(Constants.WristConstants.wristSpeed);
        } else {
          m_wrist.set(0);
        }
      } else {
        m_wrist.set(-Constants.WristConstants.wristSpeed);
      } 
    } else if( wristSpeed < -0.2 ) {
      if (wedonotwantogoupanymore) {
        if( whatButtonPressed == -1 ) {
          m_wrist.set(-Constants.WristConstants.wristSpeed);
        } else {
          m_wrist.set(0);
        }
      } else {
        m_wrist.set(Constants.WristConstants.wristSpeed);
      }
    } else {
      m_wrist.set(0);
    }
  }

  public void passedLimits() {
    if( curAngle <= upperLimit ) {
      wedonotwantogoupanymore = true;
      wedonotwantogodownanymore = false;
    } else if( curAngle >= lowerLimit ) {
      wedonotwantogodownanymore = true;
      wedonotwantogoupanymore = false;
    } else if( curAngle <= lowerLimit && curAngle >= upperLimit) {
      wedonotwantogodownanymore = false;
      wedonotwantogoupanymore = false;
    }
  }

  public void disableWristLimits() {
    wedonotwantogodownanymore = false;
    wedonotwantogoupanymore = false;
  }

  public void setClawSpeed(double speed) {
    m_claw.set(speed);
  }

  public void stopClaw() {
    m_claw.stopMotor();
  }
}
