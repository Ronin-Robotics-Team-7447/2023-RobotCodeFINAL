// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team6560.frc2023.Constants;

public class Claw extends SubsystemBase {
  CANSparkMax m_claw;
  Lights m_lights;
  String mode;

  /** Creates a new Claw. */
  public Claw(Lights l) {
    m_claw = new CANSparkMax(Constants.WristConstants.ClawID, MotorType.kBrushless);
    m_claw.setInverted(false);
    m_claw.setIdleMode(IdleMode.kBrake);
    m_claw.setSmartCurrentLimit(10);

    m_lights = l;
  }

  @Override
  public void periodic() {
    mode = m_lights.getMode();
    SmartDashboard.putNumber("Voltage of Intake", m_claw.getBusVoltage() * m_claw.getAppliedOutput());
    // This method will be called once per scheduler run
  }

  public void setClawSpeed(double speed) {
    if(mode == "cube") {
      m_claw.set(speed);
    } else if(mode == "cone") {
      m_claw.set(-speed);
    } else {
      m_claw.set(0);
    }
  }

  public void setClawSpeed1(double speed) {
    m_claw.set(speed);
  }

  public void stopClaw() {
    m_claw.stopMotor();
  }
}
