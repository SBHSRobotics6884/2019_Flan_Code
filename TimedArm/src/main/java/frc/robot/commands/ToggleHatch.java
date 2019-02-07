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

public class ToggleHatch extends Command {
  private int status;
  private double speed;
  private boolean finished;

  public ToggleHatch() {
    speed = 0.3;
    status = 0;
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    status = (status + 1) % 2;

    if(status == 0) {
      Robot.arm.moveArm(speed);
    } else if (status == 1) {
      Robot.arm.moveArm(-speed);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Timer.delay(0.2);
    Robot.arm.stopArm();
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
