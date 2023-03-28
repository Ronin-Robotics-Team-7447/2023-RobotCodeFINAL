package com.team6560.frc2023.subsystems;
import com.team6560.frc2023.Constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.SparkMaxLimitSwitch;

import static edu.wpi.first.math.util.Units.degreesToRotations;


public class Wrist2 extends SubsystemBase {
  private CANSparkMax m_Wrist;
  private SparkMaxPIDController m_pidController;
  private SparkMaxAbsoluteEncoder m_wristEncoder;
  private ArmFeedforward m_wristFeedforward;
  private TrapezoidProfile.Constraints m_constraints;
  private TrapezoidProfile.State setpoint;

  int whatButtonPressed = 0;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  double rawGoal = 0.0;
  
  NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("wrist1");
  NetworkTableEntry currentPositionNT = networkTable.getEntry("Wrist Current Position Angle");
  NetworkTableEntry wristCurrent = networkTable.getEntry("Wrist Current");

  public Wrist2() {
    // initialize motor
    m_Wrist = new CANSparkMax(Constants.WristConstants.WristID, MotorType.kBrushless);
    m_Wrist.setIdleMode(IdleMode.kBrake);
    m_Wrist.setSmartCurrentLimit(10);
    m_Wrist.setInverted(false);
    m_Wrist.restoreFactoryDefaults();

    m_wristEncoder = m_Wrist.getAbsoluteEncoder(Type.kDutyCycle);
    m_wristEncoder.setPositionConversionFactor(360);
    m_wristEncoder.setVelocityConversionFactor(360);
    m_Wrist.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
    m_Wrist.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);


    m_pidController = m_Wrist.getPIDController();
    m_pidController.setFeedbackDevice(m_wristEncoder);
    m_pidController.setSmartMotionMinOutputVelocity(0,0);
    m_pidController.setSmartMotionMaxAccel(600.0, 0);
    m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    m_pidController.setSmartMotionMaxVelocity(2000.0, 0);
    // m_pidController.setSmartMotionAllowedClosedLoopError(0.002, 0);

    m_wristFeedforward = new ArmFeedforward(0,0,0,0);

    m_Wrist.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_Wrist.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_Wrist.setSoftLimit(SoftLimitDirection.kForward, 267);
    m_Wrist.setSoftLimit(SoftLimitDirection.kReverse, 0);
    
    m_pidController.setP(0.0005, 0);
    m_pidController.setI(0, 0);
    m_pidController.setD(0, 0);
    m_pidController.setFF(0,0);
    m_pidController.setIZone(0, 0);
    m_pidController.setOutputRange(-0.3, 0.3, 0);

    currentPositionNT.setDouble(0.0);
    wristCurrent.setDouble(0.0);

    m_Wrist.setOpenLoopRampRate(0.35);

    m_Wrist.burnFlash();
  }

  @Override
  public void periodic() {
    currentPositionNT.setDouble(m_wristEncoder.getPosition());
    wristCurrent.setDouble(m_Wrist.getOutputCurrent());
    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
  }

  public void setGoal(double newGoal) {
    rawGoal = newGoal;
    m_constraints = new TrapezoidProfile.Constraints(2000, 1000);
    TrapezoidProfile.State goalPos = new TrapezoidProfile.State(rawGoal, 0);
    TrapezoidProfile.State curPos = new TrapezoidProfile.State(m_wristEncoder.getPosition(), 0);
    TrapezoidProfile profile = new TrapezoidProfile(m_constraints, goalPos, curPos);

    setpoint = profile.calculate(0.025);
    System.out.println(m_wristFeedforward.calculate(setpoint.position, setpoint.velocity));
    System.out.println(m_pidController.setReference(setpoint.position, CANSparkMax.ControlType.kSmartMotion, 0, m_wristFeedforward.calculate(setpoint.position, setpoint.velocity)));
    m_pidController.setReference(setpoint.position, CANSparkMax.ControlType.kSmartMotion, 0, m_wristFeedforward.calculate(setpoint.position, setpoint.velocity));
    // m_pidController.setReference(rawGoal, CANSparkMax.ControlType.kPosition );
  }

  public void setWristSpeed(double speed) {
    // m_Wrist.enableSoftLimit(SoftLimitDirection.kForward, false);
    // m_Wrist.enableSoftLimit(SoftLimitDirection.kReverse, false);
    m_Wrist.set(speed);
  }

  public void stop() {
    m_Wrist.stopMotor();
    // m_Wrist.enableSoftLimit(SoftLimitDirection.kForward, true);
    // m_Wrist.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }
  
  public double getEncoderPosition() {
    return m_wristEncoder.getPosition();
  }

}