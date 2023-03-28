package com.team6560.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.team6560.frc2023.Constants;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist1 extends SubsystemBase{
    double kDt = Constants.WristConstants1.trapezoidTimeFreq;

  CANSparkMax m_wrist;
  SparkMaxPIDController m_wristPID;
  SparkMaxAbsoluteEncoder m_wristEncoder;

  ArmFeedforward m_wristFeedforward;

  TrapezoidProfile.Constraints m_constraints;
  TrapezoidProfile.State setpoint;
  double rawGoal = 0.0;

  // FOR TESTING, DELETE AFTERWARDS
  double kP = 0.1;
  double kI = 1e-4;
  double kD = 0.0;
  double kIz = 0.5;
  double kMaxOutput = 0.4;
  double kMinOutput = -0.4;
  /** Creates a new arm. */
  public Wrist1() {
    m_wrist = new CANSparkMax(Constants.WristConstants1.WristID, MotorType.kBrushless);
    m_wrist.setIdleMode(IdleMode.kBrake);
    m_wrist.setSmartCurrentLimit(5);
    m_wrist.setInverted(false);

    m_wristEncoder = m_wrist.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    m_wristEncoder.setPositionConversionFactor(360);
    m_wristEncoder.setVelocityConversionFactor(360);

    m_wristPID = m_wrist.getPIDController();
    m_wristPID.setFeedbackDevice(m_wristEncoder);

    m_wristFeedforward = new ArmFeedforward(
      Constants.WristConstants1.wristkS, Constants.WristConstants1.wristkG,
      Constants.WristConstants1.wristkV, Constants.WristConstants1.wristkA);

    m_wristPID.setSmartMotionMaxAccel(200.0, 0);
    m_wristPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    m_wristPID.setSmartMotionMaxVelocity(400.0, 0);

    m_wrist.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_wrist.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_wrist.setSoftLimit(SoftLimitDirection.kForward, Constants.WristConstants1.wristUpperLimitf);
    m_wrist.setSoftLimit(SoftLimitDirection.kReverse, Constants.WristConstants1.wristLowerLimitf);

    m_wristPID.setP(kP, 0);
    m_wristPID.setI(kI, 0);
    m_wristPID.setD(kD, 0);
    m_wristPID.setIZone(kIz, 0);
    m_wristPID.setOutputRange(kMinOutput, kMaxOutput, 0);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Degree", rawGoal);
    SmartDashboard.putBoolean("Forward Soft Limit Enabled", true);
    SmartDashboard.putNumber("Forward Soft Limit", 
      m_wrist.getSoftLimit(SoftLimitDirection.kForward));
    SmartDashboard.putBoolean("Reverse Soft Limit Enabled", true);
    SmartDashboard.putNumber("Reverse Soft Limit", 
      m_wrist.getSoftLimit(SoftLimitDirection.kReverse));
    SmartDashboard.putNumber("Arm Trapezoid Max Velocity", Constants.WristConstants1.trapezoidMaxVelocity);
    SmartDashboard.putNumber("Arm Trapezoid Max Acceleration", Constants.WristConstants1.trapezoidMaxAcceleration);
  }

  @Override
  public void periodic() {
    m_wrist.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
        SmartDashboard.getBoolean("Forward Soft Limit Enabled", true));
    m_wrist.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
        SmartDashboard.getBoolean("Reverse Soft Limit Enabled", true));
    m_wrist.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
        (float) SmartDashboard.getNumber("Forward Soft Limit", Constants.WristConstants1.wristUpperLimit));
    m_wrist.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
        (float) SmartDashboard.getNumber("Reverse Soft Limit", Constants.WristConstants1.wristLowerLimit));

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", kP);
    double i = SmartDashboard.getNumber("I Gain", kI);
    double d = SmartDashboard.getNumber("D Gain", kD);
    double iz = SmartDashboard.getNumber("I Zone", kIz);
    double max = SmartDashboard.getNumber("Max Output", kMaxOutput);
    double min = SmartDashboard.getNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Arm Voltage", m_wrist.getBusVoltage());
    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      m_wristPID.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      m_wristPID.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      m_wristPID.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      m_wristPID.setIZone(iz);
      kIz = iz;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      m_wristPID.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }

    m_constraints = new TrapezoidProfile.Constraints(
        SmartDashboard.getNumber("Arm Trapezoid Max Velocity", Constants.WristConstants1.trapezoidMaxVelocity),
        SmartDashboard.getNumber("Arm Trapezoid Max Acceleration", Constants.WristConstants1.trapezoidMaxAcceleration));
    System.out.println(rawGoal);
    if (rawGoal != 0.0) {
      TrapezoidProfile.State goalPos = new TrapezoidProfile.State(rawGoal, 0);
      TrapezoidProfile.State curPos = new TrapezoidProfile.State(m_wristEncoder.getPosition(), 0);
      TrapezoidProfile profile = new TrapezoidProfile(m_constraints, goalPos, curPos);

      setpoint = profile.calculate(kDt);

      // This method will be called once per scheduler run
      m_wristPID.setReference(setpoint.position, CANSparkMax.ControlType.kPosition, 0,
          m_wristFeedforward.calculate(setpoint.position, setpoint.velocity));
    }

    SmartDashboard.putNumber("Arm SetPoint: ", rawGoal);
    SmartDashboard.putNumber("Arm Encoder Position", m_wristEncoder.getPosition());
  }

  public void setGoal(double newGoal) {
    rawGoal = newGoal;
  }

  public void stop() {
    m_wrist.stopMotor();
  }

  public void setArmSpeed(double armSpeed) {
    if (armSpeed > 0.3)
      m_wrist.set(Constants.WristConstants1.WristSpeed);
    else if (armSpeed < -0.3)
      m_wrist.set(-Constants.WristConstants1.WristSpeed);
  }
}
