/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ElevatorDock extends Command {

  int target;
  double speed;
  boolean finished;

  public ElevatorDock(int dock) {
    target = dock;
    speed = 0.6;
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (target > Robot.arm.getLevel()) {
      speed = Math.abs(speed);
    } else if(target < Robot.arm.getLevel()) {
      speed = -Math.abs(speed);
    }

    if (target != Robot.arm.getLevel()) {
      Robot.arm.moveArm(speed);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Timer.delay(0.6);
    Robot.arm.stopArm();
    Robot.arm.setLevel(target);
    finished = true;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(finished) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
