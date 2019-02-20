package frc.team3310.utility;

public class DriveTrainTest {
  public static final double STEER_NON_LINEARITY = 0.10;
  public static final double MOVE_NON_LINEARITY = 1.0;

  public static final double STICK_DEADBAND = 0.02;

  private static boolean inDeadZone(double input) {
    boolean inDeadZone;
    if (Math.abs(input) < STICK_DEADBAND) {
      inDeadZone = true;
    } else {
      inDeadZone = false;
    }
    return inDeadZone;
  }

  private static double nonlinearStickCalcPositive(double steer, double steerNonLinearity) {
    return Math.sin(Math.PI / 2.0 * steerNonLinearity * steer) / Math.sin(Math.PI / 2.0 * steerNonLinearity);
  }

  private static double nonlinearStickCalcNegative(double steer, double steerNonLinearity) {
    return Math.asin(steerNonLinearity * steer) / Math.asin(steerNonLinearity);
  }

  public static double adjustForSensitivity(double scale, double trim, double steer, int nonLinearFactor,
      double wheelNonLinearity) {
    if (inDeadZone(steer))
      return 0;

    steer += trim;
    steer *= scale;
    steer = limitValue(steer);

    int iterations = Math.abs(nonLinearFactor);
    for (int i = 0; i < iterations; i++) {
      if (nonLinearFactor > 0) {
        steer = nonlinearStickCalcPositive(steer, wheelNonLinearity);
      } else {
        steer = nonlinearStickCalcNegative(steer, wheelNonLinearity);
      }
    }
    return steer;
  }

  private static double limitValue(double value) {
    if (value > 1.0) {
      value = 1.0;
    } else if (value < -1.0) {
      value = -1.0;
    }
    return value;
  }

  public DriveTrainTest() {
  }

  public static void main(String[] args) {

    for (int i = 0; i < 10; i++) {
      double m_steerOutput = adjustForSensitivity(1.0, 0, (double) i / 10.0, -30, STEER_NON_LINEARITY);
      System.out.println(m_steerOutput);
    }
  }
}