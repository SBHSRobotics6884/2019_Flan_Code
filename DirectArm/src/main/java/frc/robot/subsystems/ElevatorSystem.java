/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class ElevatorSystem extends Subsystem {

  private SpeedController motorcontroller;
  private RobotMap map;
  private Joystick stick;

  public ElevatorSystem() {
    map = new RobotMap();
    stick = new Joystick(map.JOYSTICK_PORT);
    motorcontroller = new Spark(map.ELEVATOR_MOTOR);
  }

  public void moveArm() {
    motorcontroller.set(stick.getY() * 2);
  }

  @Override
  public void initDefaultCommand() {

  }
}
