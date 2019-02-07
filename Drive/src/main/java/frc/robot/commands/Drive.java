/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DriveSystem;

public class Drive extends Command {
  private DriveSystem drive;
  private Joystick stick;

  public Drive() {
    drive = new DriveSystem();
    stick = new Joystick(0);

    requires(drive);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    drive.driveRobot(-stick.getY(), stick.getX());
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
