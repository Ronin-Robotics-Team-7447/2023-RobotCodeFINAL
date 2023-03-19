// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team6560.frc2023.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  CANSparkMax m_wrist;
  DutyCycleEncoder wristEncoder;

  double curAngle = 0.0;
  int whatButtonPressed = 0;
  boolean wedonotwantogoupanymore = false;
  boolean wedonotwantogodownanymore = false;

  NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("wrist");
  NetworkTableEntry currentPositionNT = networkTable.getEntry("Wrist Current Position Angle");
  NetworkTableEntry upperLimitNT = networkTable.getEntry("Wrist Upper Limit");
  NetworkTableEntry lowerLimitNT = networkTable.getEntry("Wrist Lower Limit");
  NetworkTableEntry enableLimits = networkTable.getEntry("Enable Wrist Limits");

  /** Creates a new Wrist. */
  public Wrist() {
    m_wrist = new CANSparkMax(Constants.WristConstants.WristID, MotorType.kBrushless);

    m_wrist.setInverted(false);
    m_wrist.setSmartCurrentLimit(5);
    m_wrist.setIdleMode(IdleMode.kBrake);

    // wristEncoder = new DutyCycleEncoder(Constants.WristConstants.WristEncoderID);
    // wristEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    
    upperLimitNT.setDouble(Constants.WristConstants.upperLimit);
    lowerLimitNT.setDouble(Constants.WristConstants.lowerLimit);
    enableLimits.setBoolean(true);
  }

  @Override
  public void periodic() {
    // curAngle = wristEncoder.getAbsolutePosition() * 360;
    curAngle = 0;

    if(enableLimits.getBoolean(false)) {
      passedLimits();
    } else {
      disableWristLimits();
    }

    currentPositionNT.setDouble(curAngle);

    // This method will be called once per scheduler run
  }

  public boolean getEnableLimitsBoolean() {
    return enableLimits.getBoolean(false);
  }

  public void buttonPressed(int whichButton) {
    whatButtonPressed = whichButton;
  }

  public void setWristSpeed(double wristSpeed) {
    if( wristSpeed > 0.2 ) {
      if(wedonotwantogoupanymore) {
        if( whatButtonPressed == -1 ) {
          m_wrist.set(-Constants.WristConstants.wristSpeed);
        } else {
          m_wrist.set(0);
        }
      } else {
        m_wrist.set(Constants.WristConstants.wristSpeed);
      } 
    } else if( wristSpeed < -0.2 ) {
      if (wedonotwantogodownanymore) {
        if( whatButtonPressed == 1 ) {
          m_wrist.set(Constants.WristConstants.wristSpeed);
        } else {
          m_wrist.set(0);
        }
      } else {
        m_wrist.set(-Constants.WristConstants.wristSpeed);
      }
    } else {
      m_wrist.set(0);
    }
  }

  public void passedLimits() {
    if( curAngle <= upperLimitNT.getDouble(0) ) {
      wedonotwantogoupanymore = true;
      wedonotwantogodownanymore = false;
    } else if( curAngle >= lowerLimitNT.getDouble(0) ) {
      wedonotwantogodownanymore = true;
      wedonotwantogoupanymore = false;
    } else if( curAngle <= lowerLimitNT.getDouble(0) && curAngle >= upperLimitNT.getDouble(0) ) {
      wedonotwantogodownanymore = false;
      wedonotwantogoupanymore = false;
    }
  }

  public void disableWristLimits() {
    wedonotwantogodownanymore = false;
    wedonotwantogoupanymore = false;
  }

  public void stopWrist() {
    m_wrist.stopMotor();
  }

  // public void moveWristToAngle(double angle) {
  //   double secondCurAngle = Math.ceil(curAngle);
  //   passedLimits();

  //   if( secondCurAngle < angle ) {
  //     if(wedonotwantogodownanymore) {
  //       m_wrist.set(0);
  //     } else {
  //       m_wrist.set(-Constants.ArmConstants.armSpeed);
  //     } 
  //   } else if( secondCurAngle > angle ) {
  //     if (wedonotwantogoupanymore) {
  //       m_wrist.set(0);
  //     } else {
  //       m_wrist.set(Constants.ArmConstants.armSpeed);
  //     }
  //   } else {
  //     m_wrist.set(0);
  //   }
  // }
}