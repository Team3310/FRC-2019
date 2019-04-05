/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team3310.robot.RobotMap;

/**
 * Add your docs here.
 */
public class AirCompressor extends Subsystem {
  private static AirCompressor instance;
  public TalonSRX climbPump = new TalonSRX(RobotMap.CLIMB_PUMP_CAN_ID);

  Compressor compressor = new Compressor(0);
  

   	public void turnCompressorOn(){
		compressor.setClosedLoopControl(true);
	}

	public void turnCompressorOff(){
		compressor.setClosedLoopControl(false);
  }

  public void turnClimbPumpOn(){
	   climbPump.set(ControlMode.PercentOutput, 1.0);
  }

  public void turnClimbPumpOff(){
	   climbPump.set(ControlMode.PercentOutput, 0.0);
}
  
  public static AirCompressor getInstance() {
		if(instance == null) {
			instance = new AirCompressor();
		}
		return instance;
	}

  @Override
  public void initDefaultCommand() {
    
  }
}
