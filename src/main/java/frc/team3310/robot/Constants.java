package frc.team3310.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants {
    public static final double kLooperDt = 0.01;

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    // 2019 Robot Values
    public static final double kDriveWheelTrackWidthInches = 23.92;
    public static final double kDriveWheelDiameterInches = 3.8;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 0.924; // Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 60.0; // kg TODO tune
    public static final double kRobotAngularInertia = 10.0; // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 1.055; // V
    public static final double kDriveKv = 0.135; // V per rad/s
    public static final double kDriveKa = 0.012; // V per rad/s^2

    // Geometry
    // 2019 Robot Values
    public static final double kCenterToFrontBumperDistance = 15.832; // 31.664/2
    public static final double kCenterToRearBumperDistance = 15.832;
    public static final double kCenterToSideBumperDistance = 15.832;

    // Pose of the LIDAR frame w.r.t. the robot frame
    // TODO measure in CAD/on robot!
    // Dont Need we dont use Lidar (makes error in RobotStatus when removed)
    public static final double kLidarXOffset = -3.3211;
    public static final double kLidarYOffset = 0.0;
    public static final double kLidarYawAngleDegrees = 0.0;

    /* LIDAR CONSTANTS */
    public static final int kChezyLidarScanSize = 400;
    public static final int kChezyLidarNumScansToStore = 10;
    public static final String kChezyLidarPath = "/home/root/chezy_lidar";
    public static final double kChezyLidarRestartTime = 2.5;

    public static final String kLidarLogDir = "/home/lvuser/lidarLogs/";
    public static final int kNumLidarLogsToKeep = 10;
    public static final double kLidarICPTranslationEpsilon = 0.01; // convergence threshold for tx,ty
    public static final double kLidarICPAngleEpsilon = 0.01; // convergence threshold for theta

    public static final int kCameraStreamPort = 5810;

    /* LIDAR CONSTANTS */
    public static final double kScaleTrackerTimeout = 0.6;

    /* CONTROL LOOP GAINS */

    // Gearing and mechanical constants.
    public static final double kDriveDownShiftVelocity = 9.5 * 12.0; // inches per second
    public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
    public static final double kDriveUpShiftVelocity = 11.0 * 12.0; // inches per second

    public static final double kPathKX = 4.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    public static final double kDriveVelocityKp = 0.9;
    public static final double kDriveVelocityKi = 0.0;
    public static final double kDriveVelocityKd = 10.0;
    public static final double kDriveVelocityKf = 0.0;
    public static final int kDriveVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;
    public static double kDriveVelocityRampRate = 0.05; // 0.02
    public static double kDriveNominalOutput = 0.5 / 12.0;
    public static double kDriveMaxSetpoint = 17.0 * 12.0; // 17 fps

    // 2019 Motion Magic
    // PID gains for elevator velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in native units per 100ms.
    // Elevator encoder is CTRE mag encoder which is 4096 native units per
    // revolution.
    public static final double kElevatorKp = 0.0;
    public static final double kElevatorKi = 0.0; // 0.0;
    public static final double kElevatorKd = 0.0; // 4.0;
    public static final double kElevatorKf = 0.097; // 0.06;
    public static final double kElevatorJogKp = 0.1;
    public static final double kElevatorJogKd = 3.0;
    public static final double kElevatorFeedforwardNoBall = 0.197;// 33000;
    public static final double kElevatorFeedforwardWithBall = 0.297;// 33000;

    public static final int kElevatorMaxIntegralAccumulator = 500000; // todo: tune me
    public static final int kElevatorIZone = 0;
    public static final int kElevatorDeadband = 0;
    public static final int kElevatorCruiseVelocity = 10475; // 12500; //Max Velocity 10475
    public static final int kElevatorAcceleration = 15000;// 33000; //Max Velocity / Time to reach top .82
    public static final double kElevatorEpsilon = 1.0;// 33000;
    public static final double kElevatorRampRate = 0.1;

    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/179YszqnEWPWInuHUrYJnYL48LUL7LUhZrnvmNu1kujE/edit#gid=0

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    public static final int kCANTimeoutMs = 10; // use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }
}
