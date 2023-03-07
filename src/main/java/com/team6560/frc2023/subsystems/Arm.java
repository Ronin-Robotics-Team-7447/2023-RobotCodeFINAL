// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team6560.frc2023.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team6560.frc2023.Constants.*;

public class Arm extends SubsystemBase {
  CANSparkMax m_arm;
  GenericHID armJoystick;

  DutyCycleEncoder m_armEncoder;

  boolean wedonotwantogoupanymore = false;
  boolean wedonotwantogodownanymore = false;

  NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("arm");
  NetworkTableEntry currentPositionNT = networkTable.getEntry("Arm Current Position Angle");
  NetworkTableEntry upperLimitNT = networkTable.getEntry("Arm Lower Limit");
  NetworkTableEntry lowerLimitNT = networkTable.getEntry("Arm Upper Limit");
  NetworkTableEntry disableLimits = networkTable.getEntry("Disable Arm Limits");

  public Arm() {
    armJoystick = new GenericHID(Constants.TelescopeConstants.logitechID);
    m_arm = new CANSparkMax(Constants.ArmConstants.ArmID, MotorType.kBrushless);
    m_arm.setIdleMode(IdleMode.kBrake);
    m_arm.setInverted(false);

    m_armEncoder = new DutyCycleEncoder(Constants.ArmConstants.ArmEncoderID);
    m_armEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

    currentPositionNT.setDouble(0.0);
    upperLimitNT.setDouble(Constants.ArmConstants.upperLimit);
    lowerLimitNT.setDouble(Constants.ArmConstants.lowerLimit);
    disableLimits.setBoolean(false);
  }

  @Override
  public void periodic() {
    currentPositionNT.setDouble(m_armEncoder.getAbsolutePosition() * 360);
    if( disableLimits.getBoolean(false) ) {
      this.setArmSpeed(-armJoystick.getRawAxis(Constants.ArmConstants.gripYAxis));
    } else {
      passedLimits();
      this.setArmSpeed(-armJoystick.getRawAxis(Constants.ArmConstants.gripYAxis));
    }
  }

  public void setArmSpeed(double armSpeed) {
    if( armSpeed > 0.2 ) {
      if(wedonotwantogodownanymore) {
        if( armJoystick.getRawAxis(Constants.ArmConstants.gripYAxis) < 0.0 ) {
          m_arm.set(Constants.ArmConstants.armSpeed);
        } else {
          m_arm.set(0);
        }
      } else {
        m_arm.set(-Constants.ArmConstants.armSpeed);
      } 
    } else if( armSpeed < -0.2 ) {
      if (wedonotwantogoupanymore) {
        if( armJoystick.getRawAxis(Constants.ArmConstants.gripYAxis) > 0.0 ) {
          m_arm.set(-Constants.ArmConstants.armSpeed);
        } else {
          m_arm.set(0);
        }
      } else {
        m_arm.set(Constants.ArmConstants.armSpeed);
      }
    } else {
      m_arm.set(0);
    }
    }

  public void passedLimits() {
    double curAngle = currentPositionNT.getDouble(200.0);
    if( curAngle <= upperLimitNT.getDouble(0.0) ) {
      wedonotwantogoupanymore = true;
      wedonotwantogodownanymore = false;
    } else if( curAngle >= lowerLimitNT.getDouble(0.0) ) {
      wedonotwantogodownanymore = true;
      wedonotwantogoupanymore = false;
    } else if( curAngle <= lowerLimitNT.getDouble(0.0) && curAngle >= upperLimitNT.getDouble(0.0) ) {
      wedonotwantogodownanymore = false;
      wedonotwantogoupanymore = false;
    }
  }
}
