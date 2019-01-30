/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3310.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;



public class Vision extends Subsystem {
  private static Vision instance;

  public double limeArea;
  public double limeX;
  public double limeY;
  public double limeSkew;
  public boolean isLimeValid;
  public double LEDMode;
  public double camMode;

  // Vision
  public NetworkTable getLimetable() {
    return NetworkTableInstance.getDefault().getTable("limelight");
  }

  // Set the LED mode of the limelight
  /*
   * 0- Default setting in pipeline 1- Force Off 2- Force Blink 3- Force On
   */

  public void setLimeLED(int ledMode) {
    getLimetable().getEntry("ledMode").setNumber(ledMode);
  }

  // Set the camera mode
  /*
   * 0- Vision Mode 1- Driver Mode
   */
  public void setLimeCameraMode(int camMode) {
    getLimetable().getEntry("camMode").setNumber(camMode);
  }

  // Set vision pipeline
  // 0-9
  public void setPipeline(int pipeline) {
    getLimetable().getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * stream Sets limelight’s streaming mode
   * 
   * kStandard - Side-by-side streams if a webcam is attached to Limelight
   * kPiPMain - The secondary camera stream is placed in the lower-right corner of
   * the primary camera stream kPiPSecondary - The primary camera stream is placed
   * in the lower-right corner of the secondary camera stream
   * 
   * @param stream
   */
  public void setStream(String stream) {
    getLimetable().getEntry("stream").setString(stream);
  }

  // Checks if vison targets are found
  public boolean onTarget() {
    return isLimeValid;
  }

  public void updateLimelight() {
    NetworkTable limeTable = getLimetable();
    double valid = limeTable.getEntry("tv").getDouble(0);
    if (valid == 0) {
      isLimeValid = false;
    } else if (valid == 1) {
      isLimeValid = true;
    }

    limeX = limeTable.getEntry("tx").getDouble(0);
    limeY = limeTable.getEntry("ty").getDouble(0);
    limeArea = limeTable.getEntry("ta").getDouble(0);
    limeSkew = limeTable.getEntry("ts").getDouble(0);
  }

  public static Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

   
}
