// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.team6560.frc2023.Constants;
import com.team6560.frc2023.utility.NetworkTable.NtValueDisplay;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team6560.frc2023.Constants.*;

public class Arm extends SubsystemBase {
  Telescope m_telescope;

  CANSparkMax m_arm;
  GenericHID armJoystick;

  double curAngle = 0.0;
  DutyCycleEncoder m_armEncoder;
  // SparkMaxAbsoluteEncoder m_Encoder;
  double lowerLimitInCode;

  double telescopePosition = 0.0; 
  boolean wedonotwantogoupanymore = false;
  boolean wedonotwantogodownanymore = false;

  NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("arm");
  NetworkTableEntry currentPositionNT = networkTable.getEntry("Arm Current Position Angle");
  NetworkTableEntry upperLimitNT = networkTable.getEntry("Arm Upper Limit");
  NetworkTableEntry lowerLimitNT = networkTable.getEntry("Arm Lower Limit");
  NetworkTableEntry enableLimits = networkTable.getEntry("Enable Arm Limits");

  public Arm() {
    armJoystick = new GenericHID(Constants.TelescopeConstants.logitechID);
    m_arm = new CANSparkMax(Constants.ArmConstants.ArmID, MotorType.kBrushless);
    m_arm.setIdleMode(IdleMode.kBrake);
    m_arm.setSmartCurrentLimit(25);

    m_arm.setInverted(false);

    m_armEncoder = new DutyCycleEncoder(Constants.ArmConstants.ArmEncoderID);
    m_armEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    m_armEncoder.setPositionOffset(0.25);
    // m_Encoder = m_arm.getAbsoluteEncoder(Type.kDutyCycle);
    // m_Encoder.setPositionConversionFactor(360);
    // m_Encoder.setVelocityConversionFactor(360);

    currentPositionNT.setDouble(0.0);
    upperLimitNT.setDouble(Constants.ArmConstants.upperLimit);
    lowerLimitNT.setDouble(Constants.ArmConstants.lowerLimit);
    enableLimits.setBoolean(false);

    /* NtValueDisplay.ntDispTab("Arm")
      .add("Current Position Angle", this::getAngle); */
  }

  private boolean setArmSpeed = true;
  @Override
  public void periodic() {
    // telescopePosition = m_telescope.getTelescopePosition();
    curAngle = m_armEncoder.getAbsolutePosition() * 360;
    currentPositionNT.setDouble(curAngle);

    if( enableLimits.getBoolean(false) ) {
      resetLimits();
      if( setArmSpeed ) {
        this.setArmSpeed(armJoystick.getRawAxis(Constants.ArmConstants.gripYAxis));
      }
    }
     else {
      if( setArmSpeed ) {
        this.setArmSpeed(armJoystick.getRawAxis(Constants.ArmConstants.gripYAxis));
      }
      passedLimits();
    }

    lowerLimitInCode = lowerLimitNT.getDouble(Constants.ArmConstants.lowerLimit);
    // lowerLimitInCode = Constants.ArmConstants.lowerLimit;
    // if(telescopePosition >= 500 ) {
    //   lowerLimitInCode = Constants.ArmConstants.lowerLimitWhenTelescopeExtended;
    // } else {
    //   lowerLimitInCode = lowerLimitNT.getDouble(Constants.ArmConstants.lowerLimit);
    // }
  }
  
  public double getAbsoluteEncoderPosition() {
    return m_armEncoder.getAbsolutePosition() * 360;
  }

  public void setArmSpeedNoLimits(double armSpeed) {
    // enableLimits.setBoolean(true);
    m_arm.set(armSpeed);
  }

  public void setArmSpeedLimit(boolean a) {
    setArmSpeed = a;
  }

  public void setArmSpeed(double armSpeed) {
    if( armSpeed > 0.9 ) {
      if(wedonotwantogodownanymore) {
        if( armJoystick.getRawAxis(Constants.ArmConstants.gripYAxis) < 0.9 ) {
          m_arm.set(returnSpeed());
        } else {
          m_arm.set(0);
        }
      } else {
        m_arm.set(-returnSpeed());
      } 
    } else if( armSpeed < -0.9 ) {
      if (wedonotwantogoupanymore) {
        if( armJoystick.getRawAxis(Constants.ArmConstants.gripYAxis) > 0.9 ) {
          m_arm.set(-returnSpeed());
        } else {
          m_arm.set(0);
        }
      } else {
        m_arm.set(returnSpeed());
      }
    } else {
      m_arm.set(0);
    }
  }
  public double returnSpeed() {
    if( curAngle > 167 ) {
      return Constants.ArmConstants.armSpeed/4;
    } else if(curAngle > 120 ) {
      return Constants.ArmConstants.armSpeed/2;
    } else {
      return Constants.ArmConstants.armSpeed/4;
    }
  }

  // public double returnSpeed(double goal) {
  //   // as x increases, the y should increase
  //   // as x decreases, the y should decrease
  //   double speed = 0.0;
  //   double difference = Math.abs(curAngle - goal);
  //   if( curAngle-goal != 0 ) {
  //     speed = 1/difference;
  //   } else { speed = 0; }
  //   return speed;
  //   speed * (value) = 
  // }

// public double returnSpeed(double goal) {
//   return Constants.ArmConstants.armSpeed * (curAngle - goal) / 
//     (Constants.ArmConstants.lowerLimit -
//     Constants.ArmConstants.upperLimit);
// }

  public void passedLimits() {

    if( curAngle <= upperLimitNT.getDouble(0.0) ) {
      wedonotwantogoupanymore = true;
      wedonotwantogodownanymore = false;
    } else if( curAngle >= lowerLimitInCode ) {
      wedonotwantogodownanymore = true;
      wedonotwantogoupanymore = false;
    } else if( curAngle <= lowerLimitInCode && curAngle >= upperLimitNT.getDouble(0.0) ) {
      wedonotwantogodownanymore = false;
      wedonotwantogoupanymore = false;
    }
  }

  public void resetLimits() {
    wedonotwantogodownanymore = false;
    wedonotwantogoupanymore = false;
  }

  // public void moveArmToAngle(double angle) {
  //   double secondCurAngle = Math.ceil(curAngle);
  //   passedLimits();

  //   if( secondCurAngle < angle ) {
  //     if(wedonotwantogodownanymore) {
  //       m_arm.set(0);
  //     } else {
  //       m_arm.set(-returnSpeed());
  //     } 
  //   } else if( secondCurAngle > angle ) {
  //     if (wedonotwantogoupanymore) {
  //       m_arm.set(0);
  //     } else {
  //       m_arm.set(returnSpeed());
  //     }
  //   } else {
  //     m_arm.set(0);
  //   }
  // }

  public void stopArm() {
    m_arm.stopMotor();
  }
}
