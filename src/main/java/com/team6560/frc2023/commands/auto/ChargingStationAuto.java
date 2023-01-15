// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands.auto;

import com.team6560.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChargingStationAuto extends CommandBase {

  private final Drivetrain drivetrain;
  private static final double k = 0.035;

  /** Creates a new ChargingStationAuto. */
  public ChargingStationAuto(Drivetrain drivetrain) {

    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivetrain.drive(
    // ChassisSpeeds.fromFieldRelativeSpeeds(
    // chargingStationPIDController.calculate(drivetrain.getCalculatedGyroPitchRoll().getDegrees(),
    // 0),
    // 0,
    // 0,
    // drivetrain.getGyroscopeRotation()
    // )
    // );

    // if (drivetrain.getCalculatedGyroPitchRoll().getDegrees() < -3.5) {
    //   drivetrain.drive(
    //       ChassisSpeeds.fromFieldRelativeSpeeds(
    //           0.225,
    //           0,
    //           0,
    //           drivetrain.getGyroscopeRotation()));
    // } else if (drivetrain.getCalculatedGyroPitchRoll().getDegrees() > 3.5) {
    //   drivetrain.drive(
    //       ChassisSpeeds.fromFieldRelativeSpeeds(
    //           -0.225,
    //           0,
    //           0,
    //           drivetrain.getGyroscopeRotation()));
    // } else {
    //   drivetrain.drive(
    //       ChassisSpeeds.fromFieldRelativeSpeeds(
    //           0,
    //           0,
    //           0,
    //           drivetrain.getGyroscopeRotation()));
    // }
    double speed = 0.0;
    if (Math.abs(drivetrain.getCalculatedGyroPitchRoll().getDegrees()) > 1.5) {
      speed = -(drivetrain.getCalculatedGyroPitchRoll().getDegrees())* k;
    }

    drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speed,
            0,
            0,
            drivetrain.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            0,
            0,
            0,
            drivetrain.getGyroscopeRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return !DriverStation.isAutonomousEnabled();
  }
}
