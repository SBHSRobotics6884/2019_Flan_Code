/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

public class DriveSystem extends Subsystem {
   private Spark l_front, l_back, r_front, r_back;
  private SpeedControllerGroup l_motors, r_motors;
  private DifferentialDrive mdrive;

  public DriveSystem() {

    l_front = new Spark(RobotMap.LEFTFRONT);
    l_back = new Spark(RobotMap.LEFTBACK);
    r_front = new Spark(RobotMap.RIGHTFRONT);
    r_back = new Spark(RobotMap.RIGHTBACK);

    l_motors = new SpeedControllerGroup(l_front, l_back);
    r_motors = new SpeedControllerGroup(r_front, r_back);

    mdrive = new DifferentialDrive(l_motors, r_motors);
  }

  private double ramp(double inputAxis) {
    return Math.pow(inputAxis, 3);
  }

  public void driveRobot(double speed, double rotation) {
    mdrive.curvatureDrive(ramp(speed), rotation, true);
  }

  @Override
  public void initDefaultCommand() {
  }
}
