package frc.team3310.robot;

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
    // PID gains for elevator velocity loop 
    // Units: setpoint, error, and output are in native units per 100ms.
    // Elevator encoder is CTRE mag encoder which is 4096 native units per
    // revolution.
    public static final double kElevatorPositionKp = 0.075;
    public static final double kElevatorPositionKi = 0.0; 
    public static final double kElevatorPositionKd = 0.0; 
    public static final double kElevatorPositionKf = 0.0; 

    public static final double kElevatorMotionMagicKp = 0.4;
    public static final double kElevatorMotionMagicKi = 0.002; // 0.0;
    public static final double kElevatorMotionMagicKd = 0.08; // (4+4)/100;
    public static final double kElevatorMotionMagicKf = 0.12; //0.197; // 0.06;

    public static final double kElevatorFeedforwardNoBall = 0.035; //0.197;// 33000;
    public static final double kElevatorFeedforwardWithBall = 0.0; //0.297;// 33000;
    public static final int kElevatorMaxIntegralAccumulator = 500000; // todo: tune me
    public static final int kElevatorIZone = 200;
    public static final int kElevatorDeadband = 0;
    public static final int kElevatorCruiseVelocity = 10475; // 12500; //Max Velocity 10475
    public static final int kElevatorAcceleration = 15000;// 33000; //Max Velocity / Time to reach top .82
    public static final double kElevatorEpsilon = 1.0;// 33000;
    public static final int kSmoothing = 4;
    public static final double kElevatorNominalForward = 0.05;
    public static final double kElevatorNominalReverse = -0.05;
    public static final double kElevatorPeakForward = 1.0;
    public static final double kElevatorPeakReverse = -1.0;
    
    // 2019 Elevator Levels
	public static final double HOME_POSITION_INCHES = 7.5;
    public static final double ZERO_POSITION_INCHES = 7.5;
    public static final double MIN_POSITION_INCHES = HOME_POSITION_INCHES;
    public static final double MAX_POSITION_INCHES = 83.5;
    public static final double AFTER_INTAKE_POSITION_INCHES = 11.5;
    public static final double HATCH_LEVEL_1 = 19.0;
    public static final double HATCH_LEVEL_2 = 47.0;
    public static final double HATCH_LEVEL_3 = 75.0;
    public static final double BALL_LEVEL_1 = 29.5;
    public static final double BALL_LEVEL_2 = 57.5;
    public static final double BALL_LEVEL_3 = 83.5;
    public static final double BALL_LEVEL_CARGO = 42.0;
    public static final double LOADING_STATION_HATCH = 20.0;
    public static final double CLIMB = 15;

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    public static final int kCANTimeoutMs = 10; // use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

}
