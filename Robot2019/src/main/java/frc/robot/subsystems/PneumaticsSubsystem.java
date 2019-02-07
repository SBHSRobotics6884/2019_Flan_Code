/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.RobotMap;

public class PneumaticsSubsystem extends Subsystem {
  //Initializing all subsystems and other necessary classes
  public Compressor compressor;
  public DoubleSolenoid doublesolenoid;
  public static OI m_oi;
  public RobotMap map;

  //initializing variables
  public Boolean prev;
  public Boolean currentVal;
  public int toggleInt;

  public PneumaticsSubsystem() {
    //defining all necessary classes
    map = new RobotMap();
    //m_oi = new OI();
    compressor = new Compressor(0);
    doublesolenoid = new DoubleSolenoid(2, 3);

    //makes compressor work for some reason
    compressor.setClosedLoopControl(true);
  }

  public void extend() {
    doublesolenoid.set(DoubleSolenoid.Value.kForward);
  }
  public void retract() {
    doublesolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void initDefaultCommand() {
    
  }
}
