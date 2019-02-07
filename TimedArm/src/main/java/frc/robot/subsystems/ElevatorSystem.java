/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ElevatorSystem extends Subsystem {

  private Spark motorcontroller;
  private int level;

  public ElevatorSystem() {
    motorcontroller = new Spark(5);
    level = 1;
  }

  public void moveArm(double speed) {
    motorcontroller.set(speed);
  }

  public void stopArm() {
    motorcontroller.set(0);
  }

  public void setLevel(int lvl) {
    level = lvl;
  }

  public int getLevel() {
    return level;
  }

  @Override
  public void initDefaultCommand() {

  }
}
