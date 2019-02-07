/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class WristSubsystem extends Subsystem {
  // bottom, middle, and top elevator setpoints
private static final double[] kSetPoints = {1.0, 2.6};

private static final double kP = -5.0;
private static final double kI = -0.02;
private static final double kD = -2.0;

public PIDController m_pidController;
private AnalogInput m_potentiometer;
private SpeedController m_elevatorMotor;
private Joystick m_joystick;

private int m_index;
private boolean m_previousButtonValue;

public void wristSubsystem() {
  m_potentiometer = new AnalogInput(RobotMap.ARM_POT);
    m_elevatorMotor = new Spark(RobotMap.ARM_MOTOR);
    m_joystick = new Joystick(0);

    m_pidController = new PIDController(kP, kI, kD, m_potentiometer, m_elevatorMotor);
    m_pidController.setInputRange(0, 5);
}

  public void wristSetpoint() {
    boolean currentButtonValue = m_joystick.getTrigger();
    if (currentButtonValue && !m_previousButtonValue) {
      // index of the elevator setpoint wraps around.
      m_index = (m_index + 1) % kSetPoints.length;
    }
    m_previousButtonValue = currentButtonValue;

    m_pidController.setSetpoint(kSetPoints[m_index]);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
