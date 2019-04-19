package frc.team3310.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team3310.robot.RobotMap;

public class Climb extends Subsystem {
  private static Climb instance;
  private Solenoid forks;
  private Solenoid succArm;

  public static enum ForkShiftState {
    IN, OUT
  }

  public static enum ArmShiftState {
    IN, OUT
  }

  private ForkShiftState forkShiftState = ForkShiftState.IN;
  private ArmShiftState armShiftState = ArmShiftState.IN;

  private Climb() {
    try {

      forks = new Solenoid(RobotMap.FORKS_PCM_ID);
      succArm = new Solenoid(RobotMap.SUCC_ARM_PCM_ID);

    } catch (Exception e) {
      System.err.println("An error occurred in the Forks constructor");
    }
  }

  public void setForkState(ForkShiftState state) {
    forkShiftState = state;
    if (state == ForkShiftState.IN) {
      forks.set(false);
    } else if (state == ForkShiftState.OUT) {
      forks.set(true);
    }
  }

  public void setSuccArmState(ArmShiftState state) {
    armShiftState = state;
    if (state == ArmShiftState.IN) {
      succArm.set(false);
    } else if (state == ArmShiftState.OUT) {
      succArm.set(true);
    }
  }

  public void deployForks() {
    setForkState(ForkShiftState.OUT);
  }

  public void retractForks() {
    setForkState(ForkShiftState.IN);
  }

  public void deploySuccArm() {
    setSuccArmState(ArmShiftState.OUT);
  }

  public void retractSuccArm() {
    setSuccArmState(ArmShiftState.IN);
  }


  public ForkShiftState getForkShiftState() {
    return forkShiftState;
  }

  public ArmShiftState getArmShiftState() {
    return armShiftState;
  }

  @Override

  public void initDefaultCommand() {
  }
  public static Climb getInstance() {
    if (instance == null) {
      instance = new Climb();
    }
    return instance;
  }

}
