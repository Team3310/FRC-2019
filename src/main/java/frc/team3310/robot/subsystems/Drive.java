package frc.team3310.robot.subsystems;

import java.nio.file.Path;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3310.robot.Constants;
import frc.team3310.robot.OI;
import frc.team3310.robot.Robot;
import frc.team3310.robot.RobotMap;
import frc.team3310.robot.loops.Loop;
import frc.team3310.robot.planners.DriveMotionPlanner;
import frc.team3310.utility.BHRDifferentialDrive;
import frc.team3310.utility.BHRMathUtils;
import frc.team3310.utility.DriveSignal;
import frc.team3310.utility.MPSoftwarePIDController;
import frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;
import frc.team3310.utility.MPTalonPIDController;
import frc.team3310.utility.PIDParams;
import frc.team3310.utility.ReflectingCSVWriter;
import frc.team3310.utility.SoftwarePIDController;
import frc.team3310.utility.Util;
import frc.team3310.utility.lib.control.RobotStatus;
import frc.team3310.utility.lib.drivers.TalonSRXChecker;
import frc.team3310.utility.lib.drivers.TalonSRXEncoder;
import frc.team3310.utility.lib.drivers.TalonSRXFactory;
import frc.team3310.utility.lib.drivers.TalonSRXUtil;
import frc.team3310.utility.lib.geometry.Pose2d;
import frc.team3310.utility.lib.geometry.Pose2dWithCurvature;
import frc.team3310.utility.lib.geometry.Rotation2d;
import frc.team3310.utility.lib.trajectory.TrajectoryIterator;
import frc.team3310.utility.lib.trajectory.timing.TimedState;

public class Drive extends Subsystem implements Loop {
	private static Drive instance;

	public static enum DriveControlMode {
		JOYSTICK, MP_STRAIGHT, MP_TURN, PID_TURN, HOLD, MANUAL, VELOCITY_SETPOINT, CAMERA_TRACK, PATH_FOLLOWING,
		OPEN_LOOP, CAMERA_TRACK_DRIVE, MOTION_MAGIC_STRAIGHT, SPIN_MOVE
	};

	// One revolution of the wheel = Pi * D inches = 4096 ticks
	public static final double ENCODER_TICKS_TO_INCHES = 4096.0 / (Constants.kDriveWheelDiameterInches * Math.PI);
	public static final double INCHES_TO_ENCODER_TICKS_MIDDLE_DRIVE = 42.0 / 18.0 * 4096.0 / (2.38 * Math.PI);
	private static final double DRIVE_ENCODER_PPR = 4096.;
	public static final double TRACK_WIDTH_INCHES = 23.92; // 24.56; // 26.937;

	public static final double MP_STRAIGHT_T1 = 600;
	public static final double MP_STRAIGHT_T2 = 300;
	public static final double MP_TURN_T1 = 600;
	public static final double MP_TURN_T2 = 300;
	public static final double MP_MAX_TURN_T1 = 400;
	public static final double MP_MAX_TURN_T2 = 200;

	public static final double OPEN_LOOP_VOLTAGE_RAMP_HI = 0.0;
	public static final double OPEN_LOOP_VOLTAGE_RAMP_LO = 0.1;

	// Motor controllers
	private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();

	private TalonSRXEncoder leftDrive1;
	private TalonSRX leftDrive2;
	private TalonSRX leftDrive3;

	private TalonSRXEncoder rightDrive1;
	private TalonSRX rightDrive2;
	private TalonSRX rightDrive3;

	private TalonSRX middleDrive;
	private static final int kMiddleDriveMotionMagicSlot = 0;

	private BHRDifferentialDrive m_drive;

	private boolean isRed = true;
	private boolean mIsBrakeMode = false;

	private long periodMs = (long) (Constants.kLooperDt * 1000.0);

	protected Rotation2d mAngleAdjustment = Rotation2d.identity();

	public static final double STEER_NON_LINEARITY = 0.5;
	public static final double MOVE_NON_LINEARITY = 1.0;

	public static final double STICK_DEADBAND = 0.02;

	public static final double PITCH_THRESHOLD = 20;

	private int pitchWindowSize = 5;
	private int windowIndex = 0;
	private double pitchSum = 0;
	private double[] pitchAverageWindow = new double[pitchWindowSize];

	private int m_moveNonLinear = 0;
	private int m_steerNonLinear = -3;

	private double m_moveScale = 1.0;
	private double m_steerScale = 1.0;

	private double m_moveInput = 0.0;
	private double m_steerInput = 0.0;

	private double m_moveOutput = 0.0;
	private double m_steerOutput = 0.0;

	private double m_moveTrim = 0.0;
	private double m_steerTrim = 0.0;

	private boolean isFinished;
	private DriveControlMode driveControlMode = DriveControlMode.JOYSTICK;

	private static final int kPositionControlSlot = 0;
	private static final int kVelocityControlSlot = 1;
	private static final int kMotionMagicStraightSlot = 2;
	private static final int kMotionMagicTurnSlot = 3;

	private boolean isRunningMotionMagic = false;

	private MPTalonPIDController mpStraightController;
	private PIDParams mpStraightPIDParams = new PIDParams(0.1, 0, 0, 0.005, 0.03, 0.15); // 4 colsons
	// private PIDParams mpStraightPIDParams = new PIDParams(0.05, 0, 0, 0.0008,
	// 0.004, 0.03); // 4 omni
	private PIDParams mpHoldPIDParams = new PIDParams(1, 0, 0, 0.0, 0.0, 0.0);

	private MPSoftwarePIDController mpTurnController; // p i d a v g izone
	private PIDParams mpTurnPIDParams = new PIDParams(0.035, 0.000, 0.0, 0.00025, 0.00375, 0.0, 100); // 4 colson
																										// wheels
	// private PIDParams mpTurnPIDParams = new PIDParams(0.03, 0.00002, 0.4, 0.0004,
	// 0.0030, 0.0, 100); // 4 omni

	private SoftwarePIDController pidTurnController;
	private PIDParams pidTurnPIDParams = new PIDParams(0.04, 0.001, 0.4, 0, 0, 0.0, 100); // i=0.0008

	private PigeonIMU gyroPigeon;
	private double[] yprPigeon = new double[3];
	private boolean useGyroLock;
	private double gyroLockAngleDeg;
	private double kPGyro = 0.04;
	private boolean isCalibrating = false;
	private double gyroOffsetDeg = 0;

	private double mLastValidGyroAngle;
	private double mCameraVelocity = 0;
	private double kCamera = 0.04; // .7
	private double kCameraDriveClose = 0.072; // .04
	private double kCameraDriveMid = 0.043; // .04
	private double kCameraDriveFar = 0.033; // .04
	private double kCameraClose = 10;
	private double kCameraMid = 15;
	private double kCameraFar = 20;

	// Hardware states //Poofs
	private PeriodicIO mPeriodicIO;
	private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
	private DriveMotionPlanner mMotionPlanner;
	private Rotation2d mGyroOffset = Rotation2d.identity();
	public boolean mOverrideTrajectory = false;

	// Vision
	public double limeArea;
	public double lastValidLimeArea;
	public double limeX;
	public double limeY;
	public double limeSkew;
	public boolean isLimeValid;
	public double LEDMode;
	public double camMode;
	public boolean onTarget;

	// Ultrasonic
	// public Ultrasonic ultrasonic;

	public double targetDrivePositionTicks;
	public double targetMiddlePositionTicks;
	public double targetSpinAngle;
	public double spinMoveStartAngle;
	public double spinMoveStartVelocity;

	@Override
	public void onStart(double timestamp) {
		synchronized (Drive.this) {
			mpStraightController = new MPTalonPIDController(periodMs, motorControllers);
			mpStraightController.setPID(mpStraightPIDParams, kPositionControlSlot);
			mpTurnController = new MPSoftwarePIDController(periodMs, mpTurnPIDParams, motorControllers);
			pidTurnController = new SoftwarePIDController(pidTurnPIDParams, motorControllers);
		}
	}

	@Override
	public void onStop(double timestamp) {
		// TODO Auto-generated method stub
	}

	@Override
	public void onLoop(double timestamp) {
		synchronized (Drive.this) {
			DriveControlMode currentControlMode = getControlMode();
			readPeriodicInputs();

			if (currentControlMode == DriveControlMode.JOYSTICK) {
				driveWithJoystick();
			} else if (!isFinished()) {
				switch (currentControlMode) {
				case MP_STRAIGHT:
					setFinished(mpStraightController.controlLoopUpdate(getGyroAngleDeg()));
					break;
				case MP_TURN:
					setFinished(mpTurnController.controlLoopUpdate(getGyroAngleDeg()));
					break;
				case PID_TURN:
					setFinished(pidTurnController.controlLoopUpdate(getGyroAngleDeg()));
					break;
				case PATH_FOLLOWING:
					updatePathFollower();
					writePeriodicOutputs();
					break;
				case OPEN_LOOP:
					writePeriodicOutputs();
					break;
				case CAMERA_TRACK:
					updateCameraTrack();
					onTarget();
					return;
				case MOTION_MAGIC_STRAIGHT:
					if (isRunningMotionMagic == false) {
						updateDriveMotionMagic();
					}
					return;
				case SPIN_MOVE:
					updateDriveSpinMove();
				default:
					System.out.println("Unknown drive control mode: " + currentControlMode);
					break;
				}
			} else {
				// hold in current state
			}
		}
	}

	/**
	 * Configures talons for velocity control
	 */

	public void configureTalonsForSpeedControl() {
		if (!usesTalonVelocityControl(driveControlMode)) {
			leftDrive1.enableVoltageCompensation(true);
			leftDrive1.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
			leftDrive1.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
			leftDrive1.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);

			rightDrive1.enableVoltageCompensation(true);
			rightDrive1.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
			rightDrive1.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
			rightDrive1.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);

			System.out.println("configureTalonsForSpeedControl");
			leftDrive1.selectProfileSlot(kVelocityControlSlot, TalonSRXEncoder.PID_IDX);
			leftDrive1.configNominalOutputForward(Constants.kDriveNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
			leftDrive1.configNominalOutputReverse(-Constants.kDriveNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
			leftDrive1.configClosedloopRamp(Constants.kDriveVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);

			rightDrive1.selectProfileSlot(kVelocityControlSlot, TalonSRXEncoder.PID_IDX);
			rightDrive1.configNominalOutputForward(Constants.kDriveNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
			rightDrive1.configNominalOutputReverse(-Constants.kDriveNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
			rightDrive1.configClosedloopRamp(Constants.kDriveVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);
		}
	}

	private void configureMaster(TalonSRX talon) {
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
		talon.enableVoltageCompensation(true);
		talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
		talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
		talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
		talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
		talon.configNeutralDeadband(0.04, 0);
	}

	private Drive() {
		try {
			mPeriodicIO = new PeriodicIO();

			leftDrive1 = TalonSRXFactory.createTalonEncoder(RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID,
					ENCODER_TICKS_TO_INCHES, false, FeedbackDevice.CTRE_MagEncoder_Relative);
			leftDrive2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR2_CAN_ID,
					RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);
			leftDrive3 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR3_CAN_ID,
					RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);

			rightDrive1 = TalonSRXFactory.createTalonEncoder(RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID,
					ENCODER_TICKS_TO_INCHES, true, FeedbackDevice.QuadEncoder);
			rightDrive2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR2_CAN_ID,
					RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);
			rightDrive3 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR3_CAN_ID,
					RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);

			middleDrive = TalonSRXFactory.createDefaultTalon(RobotMap.DRIVE_MIDDLE_CLIMB_WHEEL);

			// ultrasonic = new Ultrasonic(RobotMap.ULTRA_SONIC_INPUT_CHANNEL,
			// RobotMap.ULTRA_SONIC_OUTPUT_CHANNEL);

			leftDrive1.setSafetyEnabled(false);
			leftDrive1.setSensorPhase(false);

			leftDrive1.setInverted(true);
			leftDrive2.setInverted(true);
			leftDrive3.setInverted(true);

			rightDrive1.setSafetyEnabled(false);
			rightDrive1.setSensorPhase(false);

			rightDrive1.setInverted(false);
			rightDrive2.setInverted(false);
			rightDrive3.setInverted(false);

			middleDrive.setNeutralMode(NeutralMode.Brake);
			middleDrive.setInverted(true);
			middleDrive.setSensorPhase(true);

			configureMaster(leftDrive1);
			configureMaster(rightDrive1);

			motorControllers.add(leftDrive1);
			motorControllers.add(rightDrive1);

			m_drive = new BHRDifferentialDrive(leftDrive1, rightDrive1);
			m_drive.setSafetyEnabled(false);

			mMotionPlanner = new DriveMotionPlanner();

			gyroPigeon = new PigeonIMU(rightDrive2);
			gyroPigeon.configFactoryDefault();
			rightDrive2.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

			middleDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
					Constants.kLongCANTimeoutMs);

			TalonSRXUtil.checkError(middleDrive.config_kP(kMiddleDriveMotionMagicSlot,
					Constants.kMiddleDriveMotionMagicKp, Constants.kLongCANTimeoutMs),
					"Could not set middle drive motion magic kp: ");

			TalonSRXUtil.checkError(middleDrive.config_kI(kMiddleDriveMotionMagicSlot,
					Constants.kMiddleDriveMotionMagicKi, Constants.kLongCANTimeoutMs),
					"Could not set middle drive motion magic ki: ");

			TalonSRXUtil.checkError(middleDrive.config_kD(kMiddleDriveMotionMagicSlot,
					Constants.kMiddleDriveMotionMagicKd, Constants.kLongCANTimeoutMs),
					"Could not set middle drive motion magic kd: ");

			TalonSRXUtil.checkError(middleDrive.config_kF(kMiddleDriveMotionMagicSlot,
					Constants.kMiddleDriveMotionMagicKf, Constants.kLongCANTimeoutMs),
					"Could not set middle drive motion magic kf: ");

			TalonSRXUtil.checkError(middleDrive.config_IntegralZone(kMiddleDriveMotionMagicSlot,
					Constants.kMiddleDriveIZone, Constants.kLongCANTimeoutMs),
					"Could not set middle drive motion magic i zone: ");

			TalonSRXUtil.checkError(
					middleDrive.configMaxIntegralAccumulator(kMiddleDriveMotionMagicSlot,
							Constants.kMiddleDriveMaxIntegralAccumulator, Constants.kLongCANTimeoutMs),
					"Could not set middle drive motion magic max integral: ");

			TalonSRXUtil.checkError(
					middleDrive.configAllowableClosedloopError(kMiddleDriveMotionMagicSlot,
							Constants.kMiddleDriveDeadband, Constants.kLongCANTimeoutMs),
					"Could not set middle drive motion magic deadband: ");

			TalonSRXUtil.checkError(middleDrive.configMotionAcceleration(Constants.kMiddleDriveAcceleration,
					Constants.kLongCANTimeoutMs), "Could not set middle drive motion magic acceleration: ");

			TalonSRXUtil.checkError(middleDrive.configMotionSCurveStrength(Constants.kMiddleDriveScurveStrength,
					Constants.kLongCANTimeoutMs), "Could not set middle drive motion magic smoothing: ");

			TalonSRXUtil.checkError(middleDrive.configMotionCruiseVelocity(Constants.kMiddleDriveCruiseVelocity,
					Constants.kLongCANTimeoutMs), "Could not set middle drive motion magic cruise velocity: ");

			TalonSRXUtil.checkError(middleDrive.configNominalOutputForward(Constants.kMiddleDriveNominalForward,
					Constants.kLongCANTimeoutMs), "Could not set nominal output forward: ");

			TalonSRXUtil.checkError(middleDrive.configNominalOutputReverse(Constants.kMiddleDriveNominalReverse,
					Constants.kLongCANTimeoutMs), "Could not set nominal output reverse: ");

			TalonSRXUtil.checkError(
					middleDrive.configPeakOutputForward(Constants.kMiddleDrivePeakForward, Constants.kLongCANTimeoutMs),
					"Could not set peak output forward: ");

			TalonSRXUtil.checkError(
					middleDrive.configPeakOutputReverse(Constants.kMiddleDrivePeakReverse, Constants.kLongCANTimeoutMs),
					"Could not set peak output reverse: ");

			TalonSRXUtil.checkError(middleDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10,
					Constants.kLongCANTimeoutMs), "Could not set peak output reverse: ");

			TalonSRXUtil.checkError(middleDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
					Constants.kLongCANTimeoutMs), "Could not set peak output reverse: ");

			TalonSRXUtil.checkError(middleDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10,
					Constants.kLongCANTimeoutMs), "Could not set peak output reverse: ");

			reloadGains();
			setBrakeMode(true);

			configMotionMagic();

		} catch (Exception e) {
			System.err.println("An error occurred in the DriveTrain constructor");
		}
	}

	private void configMotionMagic() {
		System.out.println("Start configMotionMagic");

		/* Configure the left Talon's selected sensor as local QuadEncoder */
		leftDrive1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, // Local Feedback Source
				Constants.PID_PRIMARY, // PID Slot for Source [0, 1]
				Constants.kLongCANTimeoutMs); // Configuration Timeout

		/*
		 * Configure the Remote Talon's selected sensor as a remote sensor for the right
		 * Talon
		 */
		rightDrive1.configRemoteFeedbackFilter(leftDrive1.getDeviceID(), // Device ID of Source
				RemoteSensorSource.TalonSRX_SelectedSensor, // Remote Feedback Source
				Constants.REMOTE_0, // Source number [0, 1]
				Constants.kLongCANTimeoutMs); // Configuration Timeout

		/*
		 * Configure the Pigeon IMU to the other remote slot available on the right
		 * Talon
		 */
		rightDrive1.configRemoteFeedbackFilter(gyroPigeon.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw,
				Constants.REMOTE_1, Constants.kLongCANTimeoutMs);

		System.out.println("Pigeon ID " + gyroPigeon.getDeviceID());

		/* Setup Sum signal to be used for Distance */
		rightDrive1.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kLongCANTimeoutMs);
		rightDrive1.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative,
				Constants.kLongCANTimeoutMs); // Quadrature

		rightDrive1.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, Constants.kLongCANTimeoutMs);
		rightDrive1.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative,
				Constants.kLongCANTimeoutMs); // Quadrature

		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		rightDrive1.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, Constants.PID_PRIMARY,
				Constants.kLongCANTimeoutMs);

		/* Scale Feedback by 0.5 to half the sum of Distance */
		rightDrive1.configSelectedFeedbackCoefficient(0.5, // Coefficient
				Constants.PID_PRIMARY, // PID Slot of Source
				Constants.kLongCANTimeoutMs); // Configuration Timeout

		/* Configure Remote 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index */
		// rightDrive1.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1,
		// Constants.PID_TURN,
		// Constants.kLongCANTimeoutMs);

		/* Configure Dif [Dif of both QuadEncoders] to be used for Primary PID Index */
		rightDrive1.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, Constants.PID_TURN,
				Constants.kLongCANTimeoutMs);

		/* Scale the Feedback Sensor using a coefficient */
		rightDrive1.configSelectedFeedbackCoefficient(1, Constants.PID_TURN, Constants.kLongCANTimeoutMs);

		// Set status frame rates
		rightDrive1.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kLongCANTimeoutMs);
		rightDrive1.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kLongCANTimeoutMs);
		rightDrive1.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kLongCANTimeoutMs);
		rightDrive1.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kLongCANTimeoutMs);
		leftDrive1.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kLongCANTimeoutMs);
		gyroPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 5, Constants.kLongCANTimeoutMs);

		/* Configure neutral deadband */
		leftDrive1.configNeutralDeadband(0.001, Constants.kLongCANTimeoutMs);
		rightDrive1.configNeutralDeadband(0.001, Constants.kLongCANTimeoutMs);

		// Set Motion Magic accel, velocity terms
		rightDrive1.configMotionAcceleration(Constants.kDriveAcceleration, Constants.kLongCANTimeoutMs);
		rightDrive1.configMotionCruiseVelocity(Constants.kDriveCruiseVelocity, Constants.kLongCANTimeoutMs);
		// rightDrive1.configMotionSCurveStrength(Constants.kDriveScurveStrength,
		// Constants.kLongCANTimeoutMs);

		/**
		 * Max out the peak output (for all modes). However you can limit the output of
		 * a given PID object with configClosedLoopPeakOutput().
		 */
		leftDrive1.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
		leftDrive1.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);

		rightDrive1.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
		rightDrive1.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);

		// Set PID gains for straight
		rightDrive1.config_kP(kMotionMagicStraightSlot, Constants.kDriveMotionMagicStraightKp,
				Constants.kLongCANTimeoutMs);
		rightDrive1.config_kI(kMotionMagicStraightSlot, Constants.kDriveMotionMagicStraightKi,
				Constants.kLongCANTimeoutMs);
		rightDrive1.config_kD(kMotionMagicStraightSlot, Constants.kDriveMotionMagicStraightKd,
				Constants.kLongCANTimeoutMs);
		rightDrive1.config_kF(kMotionMagicStraightSlot, Constants.kDriveMotionMagicStraightKf,
				Constants.kLongCANTimeoutMs);
		rightDrive1.config_IntegralZone(kMotionMagicStraightSlot, Constants.kDriveMotionMagicStraightIZone,
				Constants.kLongCANTimeoutMs);
		rightDrive1.configClosedLoopPeakOutput(kMotionMagicStraightSlot, 1.0, Constants.kLongCANTimeoutMs);
		// rightDrive1.configMaxIntegralAccumulator(kMotionMagicStraightSlot,
		// Constants.kDriveMotionMagicStraightMaxIntegralAccumulator,
		// Constants.kLongCANTimeoutMs);
		// rightDrive1.configAllowableClosedloopError(kMotionMagicStraightSlot,
		// Constants.kDriveMotionMagicStraightDeadband, Constants.kLongCANTimeoutMs);

		// Set PID gains for turn
		rightDrive1.config_kP(kMotionMagicTurnSlot, Constants.kDriveMotionMagicTurnKp, Constants.kLongCANTimeoutMs);
		rightDrive1.config_kI(kMotionMagicTurnSlot, Constants.kDriveMotionMagicTurnKi, Constants.kLongCANTimeoutMs);
		rightDrive1.config_kD(kMotionMagicTurnSlot, Constants.kDriveMotionMagicTurnKd, Constants.kLongCANTimeoutMs);
		rightDrive1.config_kF(kMotionMagicTurnSlot, Constants.kDriveMotionMagicTurnKf, Constants.kLongCANTimeoutMs);
		rightDrive1.config_IntegralZone(kMotionMagicTurnSlot, Constants.kDriveMotionMagicTurnIZone,
				Constants.kLongCANTimeoutMs);
		rightDrive1.configClosedLoopPeakOutput(kMotionMagicTurnSlot, 1.0, Constants.kLongCANTimeoutMs);
		// rightDrive1.configMaxIntegralAccumulator(kMotionMagicTurnSlot,
		// Constants.kDriveMotionMagicTurnMaxIntegralAccumulator,
		// Constants.kLongCANTimeoutMs);
		// rightDrive1.configAllowableClosedloopError(kMotionMagicTurnSlot,
		// Constants.kDriveMotionMagicTurnDeadband, Constants.kLongCANTimeoutMs);

		/**
		 * 1ms per loop. PID loop can be slowed down if need be. For example, - if
		 * sensor updates are too slow - sensor deltas are very small per update, so
		 * derivative error never gets large enough to be useful. - sensor movement is
		 * very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		rightDrive1.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kLongCANTimeoutMs);
		rightDrive1.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kLongCANTimeoutMs);

		/**
		 * configAuxPIDPolarity(boolean invert, int timeoutMs) false means talon's local
		 * output is PID0 + PID1, and other side Talon is PID0 - PID1 true means talon's
		 * local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		rightDrive1.configAuxPIDPolarity(false, Constants.kLongCANTimeoutMs);

		rightDrive1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);

		System.out.println("End configMotionMagic");
	}

	public static Drive getInstance() {
		if (instance == null) {
			instance = new Drive();
		}
		return instance;
	}

	// Encoder and Gryo Setup
	public static double rotationsToInches(double rotations) {
		return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
	}

	public double getRightPositionInches() {
		return rightDrive1.getPositionWorld();
	}

	public double getLeftPositionInches() {
		return leftDrive1.getPositionWorld();
	}

	private static double radiansPerSecondToTicksPer100ms(double rad_s) {
		return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
	}

	public double getLeftEncoderRotations() {
		return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
	}

	public double getRightEncoderRotations() {
		return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
	}

	public double getLeftWheelRotations() {
		return getLeftEncoderRotations();
	}

	public double getRightWheelRotations() {
		return getRightEncoderRotations();
	}

	public double getLeftWheelDistance() {
		return rotationsToInches(getLeftWheelRotations());
	}

	public double getRightWheelDistance() {
		return rotationsToInches(getRightWheelRotations());
	}

	public double getRightVelocityNativeUnits() {
		return mPeriodicIO.right_velocity_ticks_per_100ms;
	}

	public double getRightLinearVelocity() {
		return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
	}

	public double getLeftVelocityNativeUnits() {
		return mPeriodicIO.left_velocity_ticks_per_100ms;
	}

	public double getLeftLinearVelocity() {
		return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
	}

	public double getLinearVelocity() {
		return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
	}

	public double getAngularVelocity() {
		return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
	}

	public synchronized void resetEncoders() {
		rightDrive1.setPosition(0);
		leftDrive1.setPosition(0);
	}

	public void calibrateGyro() {
		gyroPigeon.enterCalibrationMode(CalibrationMode.Temperature, TalonSRXEncoder.TIMEOUT_MS);
	}

	public void endGyroCalibration() {
		if (isCalibrating == true) {
			isCalibrating = false;
		}
	}

	public void setGyroOffset(double offsetDeg) {
		gyroOffsetDeg = offsetDeg;
	}

	public synchronized Rotation2d getGyroAngle() {
		return mAngleAdjustment.rotateBy(Rotation2d.fromDegrees(-getGyroAngleDeg()));
	}

	public synchronized void setGyroAngle(Rotation2d adjustment) {
		resetGyro();
		mAngleAdjustment = adjustment;
	}

	public synchronized double getGyroAngleDeg() {
		gyroPigeon.getYawPitchRoll(yprPigeon);
		return -yprPigeon[0] + gyroOffsetDeg;
	}

	public synchronized double getGyroPitchAngle() {
		gyroPigeon.getYawPitchRoll(yprPigeon);
		return yprPigeon[2];
	}

	public boolean checkPitchAngle() {
		double pitchAngle = Math.abs(getGyroPitchAngle());
		if (pitchAngle > 10) {
			return true;
		}
		return false;
	}

	public synchronized void resetGyro() {
		gyroPigeon.setYaw(0, TalonSRXEncoder.TIMEOUT_MS);
		gyroPigeon.setFusedHeading(0, TalonSRXEncoder.TIMEOUT_MS);
	}

	public synchronized Rotation2d getHeading() {
		return mPeriodicIO.gyro_heading;
	}

	public synchronized void setHeading(Rotation2d heading) {
		System.out.println("SET HEADING: " + heading.getDegrees());

		mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(gyroPigeon.getFusedHeading()).inverse());
		System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

		mPeriodicIO.gyro_heading = heading;
	}

	public void zeroSensors() {
		resetEncoders();
		resetGyro();
		resetMiddleEncoder();
	}
	// End

	// Auto Setup
	private void setOpenLoopVoltageRamp(double timeTo12VSec) {
		leftDrive1.configOpenloopRamp(timeTo12VSec, TalonSRXEncoder.TIMEOUT_MS);
		rightDrive1.configOpenloopRamp(timeTo12VSec, TalonSRXEncoder.TIMEOUT_MS);
	}

	public void setStraightMP(double distanceInches, double maxVelocity, boolean useGyroLock, boolean useAbsolute,
			double desiredAbsoluteAngle) {
		double yawAngle = useAbsolute ? BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), desiredAbsoluteAngle)
				: getGyroAngleDeg();
		mpStraightController.setPID(mpStraightPIDParams, kPositionControlSlot);
		mpStraightController.setPIDSlot(kPositionControlSlot);
		mpStraightController.setMPStraightTarget(0, distanceInches, maxVelocity, MP_STRAIGHT_T1, MP_STRAIGHT_T2,
				useGyroLock, yawAngle, true);
		setControlMode(DriveControlMode.MP_STRAIGHT);
	}

	public void setRelativeTurnMP(double relativeTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(getGyroAngleDeg(), relativeTurnAngleDeg + getGyroAngleDeg(), turnRateDegPerSec,
				MP_TURN_T1, MP_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		setControlMode(DriveControlMode.MP_TURN);
	}

	public void setRelativeMaxTurnMP(double relativeTurnAngleDeg, double turnRateDegPerSec,
			MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(getGyroAngleDeg(), relativeTurnAngleDeg + getGyroAngleDeg(), turnRateDegPerSec,
				MP_MAX_TURN_T1, MP_MAX_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		setControlMode(DriveControlMode.MP_TURN);
	}

	public void setAbsoluteTurnMP(double absoluteTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(getGyroAngleDeg(),
				BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), absoluteTurnAngleDeg), turnRateDegPerSec,
				MP_TURN_T1, MP_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		setControlMode(DriveControlMode.MP_TURN);
	}

	public synchronized void setGyroLock(boolean useGyroLock, boolean snapToAbsolute0or180) { // Delete
		if (snapToAbsolute0or180) {
			gyroLockAngleDeg = BHRMathUtils.adjustAccumAngleToClosest180(getGyroAngleDeg());
		} else {
			gyroLockAngleDeg = getGyroAngleDeg();
		}
		this.useGyroLock = useGyroLock;
	}
	// End

	// MP Setup
	/**
	 * Start up velocity mode. This sets the drive train in high gear as well.
	 * 
	 * @param left_inches_per_sec
	 * @param right_inches_per_sec
	 */
	/**
	 * Check if the drive talons are configured for velocity control
	 */
	protected static boolean usesTalonVelocityControl(DriveControlMode state) {
		if (state == DriveControlMode.VELOCITY_SETPOINT || state == DriveControlMode.PATH_FOLLOWING
				|| state == DriveControlMode.CAMERA_TRACK) {
			return true;
		}
		return false;
	}

	/**
	 * Check if the drive talons are configured for position control
	 */
	protected static boolean usesTalonPositionControl(DriveControlMode state) {
		if (state == DriveControlMode.MP_STRAIGHT || state == DriveControlMode.MP_TURN
				|| state == DriveControlMode.HOLD) {
			return true;
		}
		return false;
	}

	public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		configureTalonsForSpeedControl();
		driveControlMode = DriveControlMode.VELOCITY_SETPOINT;
		updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
	}

	public synchronized void setVelocityNativeUnits(double left_velocity_ticks_per_100ms,
			double right_velocity_ticks_per_100ms) {
		leftDrive1.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
				mPeriodicIO.left_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.left_accel / 1023.0);
		rightDrive1.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
				mPeriodicIO.right_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.right_accel / 1023.0);
	}

	/**
	 * Adjust Velocity setpoint (if already in velocity mode)
	 * 
	 * @param left_inches_per_sec
	 * @param right_inches_per_sec
	 */
	private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		if (usesTalonVelocityControl(driveControlMode)) {
			final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
			final double maxSetpoint = Constants.kDriveMaxSetpoint;
			final double scale = max_desired > maxSetpoint ? maxSetpoint / max_desired : 1.0;

			leftDrive1.setVelocityWorld(left_inches_per_sec * scale);
			rightDrive1.setVelocityWorld(right_inches_per_sec * scale);

		} else {
			System.out.println("Hit a bad velocity control state");
			leftDrive1.set(ControlMode.Velocity, 0);
			rightDrive1.set(ControlMode.Velocity, 0);
		}
	}

	public synchronized void setOpenLoop(DriveSignal signal) {
		if (driveControlMode != DriveControlMode.OPEN_LOOP) {
			setBrakeMode(false);

			System.out.println("Switching to open loop");
			System.out.println(signal);
			driveControlMode = DriveControlMode.OPEN_LOOP;
			rightDrive1.configNeutralDeadband(0.04, 0);
			leftDrive1.configNeutralDeadband(0.04, 0);
		}
		mPeriodicIO.left_demand = signal.getLeft();
		mPeriodicIO.right_demand = signal.getRight();
		mPeriodicIO.left_feedforward = 0.0;
		mPeriodicIO.right_feedforward = 0.0;
	}

	/**
	 * Configures talons for velocity control
	 */
	public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
		mPeriodicIO.left_demand = signal.getLeft();
		mPeriodicIO.right_demand = signal.getRight();
		mPeriodicIO.left_feedforward = feedforward.getLeft();
		mPeriodicIO.right_feedforward = feedforward.getRight();
	}

	public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
		if (mMotionPlanner != null) {
			mOverrideTrajectory = false;
			mMotionPlanner.reset();
			mMotionPlanner.setTrajectory(trajectory);

			// We entered a velocity control state.
			setBrakeMode(true);
			leftDrive1.selectProfileSlot(kVelocityControlSlot, 0);
			rightDrive1.selectProfileSlot(kVelocityControlSlot, 0);
			leftDrive1.configNeutralDeadband(0.0, 0);
			rightDrive1.configNeutralDeadband(0.0, 0);

			setControlMode(DriveControlMode.PATH_FOLLOWING);
		}
	}

	public boolean isDoneWithTrajectory() {
		if (mMotionPlanner == null) {
			return false;
		}
		return mMotionPlanner.isDone() || mOverrideTrajectory == true;
	}

	public void overrideTrajectory(boolean value) {
		mOverrideTrajectory = value;
	}

	private void updatePathFollower() {
		if (driveControlMode == DriveControlMode.PATH_FOLLOWING) {
			final double now = Timer.getFPGATimestamp();

			DriveMotionPlanner.Output output = mMotionPlanner.update(now,
					RobotStatus.getInstance().getFieldToVehicle(now));

			// DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0,
			// demand.right_feedforward_voltage / 12.0);

			mPeriodicIO.error = mMotionPlanner.error();
			mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

			if (!mOverrideTrajectory) {
				setVelocity(
						new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity),
								radiansPerSecondToTicksPer100ms(output.right_velocity)),
						new DriveSignal(output.left_feedforward_voltage / 12.0,
								output.right_feedforward_voltage / 12.0));

				mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
				mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
			} else {
				setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
				mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
			}
		} else {
			DriverStation.reportError("Drive is not in path following state", false);
		}
	}

	public synchronized void reloadGains() {
		leftDrive1.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
		leftDrive1.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
		leftDrive1.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
		leftDrive1.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
		leftDrive1.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone,
				Constants.kLongCANTimeoutMs);

		rightDrive1.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
		rightDrive1.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
		rightDrive1.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
		rightDrive1.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
		rightDrive1.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone,
				Constants.kLongCANTimeoutMs);

	}

	public void writeToLog() {
	}

	public synchronized void readPeriodicInputs() {
		double prevLeftTicks = mPeriodicIO.left_position_ticks;
		double prevRightTicks = mPeriodicIO.right_position_ticks;
		mPeriodicIO.left_position_ticks = leftDrive1.getSelectedSensorPosition(0);
		mPeriodicIO.right_position_ticks = rightDrive1.getSelectedSensorPosition(0);
		mPeriodicIO.left_velocity_ticks_per_100ms = leftDrive1.getSelectedSensorVelocity(0);
		mPeriodicIO.right_velocity_ticks_per_100ms = rightDrive1.getSelectedSensorVelocity(0);
		mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(gyroPigeon.getFusedHeading()).rotateBy(mGyroOffset);

		double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / 4096.0) * Math.PI;
		if (deltaLeftTicks > 0.0) {
			mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
		} else {
			mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
		}

		double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / 4096.0) * Math.PI;
		if (deltaRightTicks > 0.0) {
			mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
		} else {
			mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;

		}

		if (mCSVWriter != null) {
			mCSVWriter.add(mPeriodicIO);
		}

		// System.out.println("control state: " + mDriveControlState + ", left: " +
		// mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
	}

	public synchronized void writePeriodicOutputs() {
		if (driveControlMode == DriveControlMode.OPEN_LOOP) {
			leftDrive1.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
			rightDrive1.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
		} else {
			leftDrive1.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
					mPeriodicIO.left_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.left_accel / 1023.0);
			rightDrive1.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
					mPeriodicIO.right_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.right_accel / 1023.0);
		}
	}
	// End

	// Drive
	public synchronized DriveControlMode getControlMode() {
		return driveControlMode;
	}

	public synchronized void setControlMode(DriveControlMode controlMode) {
		this.driveControlMode = controlMode;
		if (controlMode == DriveControlMode.HOLD) {
			mpStraightController.setPID(mpHoldPIDParams, kPositionControlSlot);
			leftDrive1.setPosition(0);
			leftDrive1.set(ControlMode.Position, 0);
			rightDrive1.setPosition(0);
			rightDrive1.set(ControlMode.Position, 0);
		}
		setFinished(false);
	}

	public synchronized void setSpeed(double speed) {
		if (speed == 0) {
			setControlMode(DriveControlMode.JOYSTICK);
		} else {
			setControlMode(DriveControlMode.MANUAL);
			rightDrive1.set(ControlMode.PercentOutput, speed);
			leftDrive1.set(ControlMode.PercentOutput, speed);
		}
	}

	public void setDriveHold(boolean status) {
		if (status) {
			setControlMode(DriveControlMode.HOLD);
		} else {
			setControlMode(DriveControlMode.JOYSTICK);
		}
	}

	public void driveForwardClimb(double speed) {
		middleDrive.set(ControlMode.PercentOutput, speed);
	}

	public synchronized void driveWithJoystick() {
		if (m_drive == null)
			return;

		boolean cameraTrackTapeButton = OI.getInstance().getDriverController().getRightTrigger().get();

		m_moveInput = OI.getInstance().getDriverController().getLeftYAxis();
		m_steerInput = -OI.getInstance().getDriverController().getRightXAxis();

		m_moveOutput = adjustForSensitivity(m_moveScale, m_moveTrim, m_moveInput, m_moveNonLinear, MOVE_NON_LINEARITY);
		m_steerOutput = adjustForSensitivity(m_steerScale, m_steerTrim, m_steerInput, m_steerNonLinear,
				STEER_NON_LINEARITY);

		if (useGyroLock) {
			double yawError = gyroLockAngleDeg - getGyroAngleDeg();
			m_steerOutput = kPGyro * yawError;
		}

		double pitchAngle = updatePitchWindow();
		if (Math.abs(pitchAngle) > PITCH_THRESHOLD) {
			m_moveOutput = Math.signum(pitchAngle) * -1.0;
			m_steerOutput = 0;
			System.out.println("Pitch Treshhold 2 angle = " + pitchAngle);
		}

		if (cameraTrackTapeButton) {
			setPipeline(0);
			setLimeLED(0);
			updateLimelight();
			double cameraSteer = 0;
			mLastValidGyroAngle = getGyroAngleDeg();
			if (isLimeValid) {
				double kCameraDrive = kCameraDriveClose;
				if (limeX <= kCameraClose) {
					kCameraDrive = kCameraDriveClose;
					// System.out.println(" Close Valid lime angle = " + limeX);
				} else if (limeX < kCameraMid) {
					kCameraDrive = kCameraDriveMid;
					// System.out.println("Mid Valid lime angle = " + limeX);
				} else if (limeX < kCameraFar) {
					kCameraDrive = kCameraDriveFar;
					// System.out.println("Far Valid lime angle = " + limeX);
				}
				cameraSteer = limeX * kCameraDrive;
				// System.out.println("Valid lime angle = " + kCameraDrive);
			} else {
				// System.out.println("In Valid lime angle = " + limeX);
				cameraSteer = -m_steerOutput;
			}
			m_steerOutput = -cameraSteer;
		}

		m_drive.arcadeDrive(-m_moveOutput, -m_steerOutput);

	}

	public boolean isBrakeMode() {
		return mIsBrakeMode;
	}

	public synchronized void setBrakeMode(boolean on) {
		if (mIsBrakeMode != on) {
			mIsBrakeMode = on;
			rightDrive1.setNeutralMode(NeutralMode.Brake);
			rightDrive2.setNeutralMode(NeutralMode.Brake);
			rightDrive3.setNeutralMode(NeutralMode.Brake);
			leftDrive1.setNeutralMode(NeutralMode.Brake);
			leftDrive2.setNeutralMode(NeutralMode.Brake);
			leftDrive3.setNeutralMode(NeutralMode.Brake);
		}
	}

	private double updatePitchWindow() {
		double lastPitchAngle = pitchAverageWindow[windowIndex];
		double currentPitchAngle = getGyroPitchAngle();
		pitchAverageWindow[windowIndex] = currentPitchAngle;
		pitchSum = pitchSum - lastPitchAngle + currentPitchAngle;

		windowIndex++;
		if (windowIndex == pitchWindowSize) {
			windowIndex = 0;
		}

		return pitchSum / pitchWindowSize;
	}

	private boolean inDeadZone(double input) {
		boolean inDeadZone;
		if (Math.abs(input) < STICK_DEADBAND) {
			inDeadZone = true;
		} else {
			inDeadZone = false;
		}
		return inDeadZone;
	}

	public double adjustForSensitivity(double scale, double trim, double steer, int nonLinearFactor,
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

	private double limitValue(double value) {
		if (value > 1.0) {
			value = 1.0;
		} else if (value < -1.0) {
			value = -1.0;
		}
		return value;
	}

	private double nonlinearStickCalcPositive(double steer, double steerNonLinearity) {
		return Math.sin(Math.PI / 2.0 * steerNonLinearity * steer) / Math.sin(Math.PI / 2.0 * steerNonLinearity);
	}

	private double nonlinearStickCalcNegative(double steer, double steerNonLinearity) {
		return Math.asin(steerNonLinearity * steer) / Math.asin(steerNonLinearity);
	}

	public synchronized boolean isFinished() {
		return isFinished;
	}

	public synchronized void setFinished(boolean isFinished) {
		this.isFinished = isFinished;
	}
	// End

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

	public void setStream(String stream) {
		getLimetable().getEntry("stream").setString(stream);
	}

	// Checks if vison targets are found
	public boolean isValid() {
		return isLimeValid;
	}

	public void updateLimelight() {
		NetworkTable limeTable = getLimetable();
		limeX = limeTable.getEntry("tx").getDouble(0);
		limeY = limeTable.getEntry("ty").getDouble(0);
		limeArea = limeTable.getEntry("ta").getDouble(0);
		limeSkew = limeTable.getEntry("ts").getDouble(0);
		double valid = limeTable.getEntry("tv").getDouble(0);
		if (valid == 0) { // || limeArea > 38
			isLimeValid = false;
		} else if (valid == 1) {
			isLimeValid = true;
			lastValidLimeArea = limeArea;
		}
	}

	/**
	 * Called periodically when the robot is in camera track mode.
	 */
	private void updateCameraTrack() {
		updateLimelight();
		double deltaVelocity = 0;
		mLastValidGyroAngle = getGyroAngleDeg();
		if (isLimeValid) {
			deltaVelocity = limeX * kCamera * mCameraVelocity;
			// System.out.println("Valid lime angle = " + limeX);
		} else {
			deltaVelocity = (getGyroAngleDeg() - mLastValidGyroAngle) * kCamera * mCameraVelocity;
			// System.out.println("In Valid lime angle = " + limeX);
		}
		// setVelocityNativeUnits(mCameraVelocity + deltaVelocity, mCameraVelocity -
		// deltaVelocity);
		updateVelocitySetpoint(mCameraVelocity + deltaVelocity, mCameraVelocity - deltaVelocity);
	}

	/**
	 * Configures the drivebase to drive a path. Used for autonomous driving
	 * 
	 * @see Path
	 */
	public synchronized void setCameraTrack(double velocityScale) {
		// double straightVelocity = (mPeriodicIO.left_velocity_ticks_per_100ms
		// + mPeriodicIO.right_velocity_ticks_per_100ms) / 2;
		double straightVelocity = velocityScale * (leftDrive1.getVelocityWorld() + rightDrive1.getVelocityWorld()) / 2;
		setFinished(false);
		// configureTalonsForSpeedControl();
		driveControlMode = DriveControlMode.CAMERA_TRACK;
		mLastValidGyroAngle = getGyroAngleDeg();
		mCameraVelocity = straightVelocity;
	}

	public boolean onTarget() {
		if (limeX < 5) {
			onTarget = true;
		} else {
			onTarget = false;
		}
		return onTarget;
	}

	// End

	public double getPeriodMs() {
		return periodMs;
	}

	public boolean isRed() {
		return isRed;
	}

	public void setIsRed(boolean status) {
		isRed = status;
	}

	@Override
	public void initDefaultCommand() {
	}

	public synchronized void startLogging() {
		if (mCSVWriter == null) {
			mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
		}
	}

	public synchronized void stopLogging() {
		if (mCSVWriter != null) {
			mCSVWriter.flush();
			mCSVWriter = null;
		}
	}

	public synchronized void setDriveMotionMagic(double positionInches, double turnDegrees) {
		targetDrivePositionTicks = getDriveEncoderTicks(positionInches);
		System.out.println("MM Target Ticks = " + targetDrivePositionTicks);
		driveControlMode = DriveControlMode.MOTION_MAGIC_STRAIGHT;
		isRunningMotionMagic = false;
	}

	public synchronized void updateDriveMotionMagic() {
		isRunningMotionMagic = true;
		rightDrive1.selectProfileSlot(kMotionMagicStraightSlot, Constants.PID_PRIMARY);
		rightDrive1.selectProfileSlot(kMotionMagicTurnSlot, Constants.PID_TURN);

		leftDrive1.getSensorCollection().setQuadraturePosition(0, 0);
		rightDrive1.getSensorCollection().setQuadraturePosition(0, 0);

		double currentGyro = rightDrive1.getSelectedSensorPosition(1);
		System.out.println("MM Gyro from Talon = " + currentGyro + ", Fused heading from pidgeon = "
				+ gyroPigeon.getFusedHeading() + ", gyro angle = " + getGyroAngleDeg());
		System.out.println("MM Target Ticks = " + targetDrivePositionTicks);

		rightDrive1.set(ControlMode.MotionMagic, targetDrivePositionTicks, DemandType.AuxPID, 0);
		// rightDrive1.set(ControlMode.MotionMagic, targetDrivePositionTicks);
		// rightDrive1.set(ControlMode.Velocity, 5000, DemandType.AuxPID, 0);
		leftDrive1.follow(rightDrive1, FollowerType.AuxOutput1);
	}

	public synchronized void setDriveSpinMove(double turnDegrees) {
		targetSpinAngle = turnDegrees;
		spinMoveStartAngle = getGyroAngleDeg();
		spinMoveStartVelocity = (rightDrive1.getSelectedSensorVelocity() + leftDrive1.getSelectedSensorVelocity()) / 2;
		driveControlMode = DriveControlMode.SPIN_MOVE;
		rightDrive1.selectProfileSlot(kVelocityControlSlot, Constants.PID_PRIMARY);
		leftDrive1.selectProfileSlot(kVelocityControlSlot, Constants.PID_PRIMARY);
	}

	public synchronized void updateDriveSpinMove() {
		double deltaAngle = getGyroAngleDeg() - spinMoveStartAngle;

		double cosA = Math.cos(Math.toRadians(deltaAngle));
		double sinA = Math.sin(Math.toRadians(deltaAngle));
		
		double rightVelocity = spinMoveStartVelocity * (cosA + sinA); 
		double leftVelocity = spinMoveStartVelocity * (cosA - sinA); 
		
		rightDrive1.set(ControlMode.Velocity, rightVelocity);
		leftDrive1.set(ControlMode.Velocity, leftVelocity);
	}

	private int getDriveEncoderTicks(double positionInches) {
		return (int) (positionInches * ENCODER_TICKS_TO_INCHES);
	}

	public synchronized boolean hasFinishedDriveMotionMagic() {
		return Util.epsilonEquals(rightDrive1.getActiveTrajectoryPosition(), targetDrivePositionTicks, 5);
	}

	public synchronized double getDriveMotionMagicPosition() {
		return rightDrive1.getActiveTrajectoryPosition();
	}

	public synchronized void setMiddleDriveMotionMagicPosition(double positionInches) {
		middleDrive.selectProfileSlot(kMiddleDriveMotionMagicSlot, 0);
		targetMiddlePositionTicks = getMiddleEncoderTicks(positionInches);
		middleDrive.set(ControlMode.MotionMagic, targetMiddlePositionTicks);
	}

	public void resetMiddleEncoder() {
		middleDrive.setSelectedSensorPosition(0, 0, 100);
	}

	private int getMiddleEncoderTicks(double positionInches) {
		return (int) (positionInches * INCHES_TO_ENCODER_TICKS_MIDDLE_DRIVE);
	}

	public synchronized double getMiddleEncoderInches() {
		double position_ticks = middleDrive.getSelectedSensorPosition(0);
		return position_ticks / INCHES_TO_ENCODER_TICKS_MIDDLE_DRIVE;
	}

	public synchronized boolean hasFinishedTrajectory() {
		return Util.epsilonEquals(middleDrive.getActiveTrajectoryPosition(), targetMiddlePositionTicks, 5);
	}

	// Ultrasonic
	// public void setAutomatic() {
	// ultrasonic.setAutomaticMode(true); // turns on automatic mode
	// }

	// public double ultrasonicDistance() {
	// return ultrasonic.getRangeInches(); // reads the range on the ultrasonic
	// sensor
	// }

	public static class PeriodicIO {
		// INPUTS
		public int left_position_ticks;
		public int right_position_ticks;
		public double left_distance;
		public double right_distance;
		public int left_velocity_ticks_per_100ms;
		public int right_velocity_ticks_per_100ms;
		public Rotation2d gyro_heading = Rotation2d.identity();
		public Pose2d error = Pose2d.identity();

		// OUTPUTS
		public double left_demand;
		public double right_demand;
		public double left_accel;
		public double right_accel;
		public double left_feedforward;
		public double right_feedforward;
		public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(
				Pose2dWithCurvature.identity());
	}

	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putNumber("Drive Right Position Inches", rightDrive1.getPositionWorld());
				SmartDashboard.putNumber("Drive Left Position Inches", leftDrive1.getPositionWorld());
				SmartDashboard.putNumber("Drive Right Velocity InPerSec", rightDrive1.getVelocityWorld());
				SmartDashboard.putNumber("Drive Left Velocity InPerSec", leftDrive1.getVelocityWorld());
				SmartDashboard.putNumber("Drive Left 1 Amps", leftDrive1.getOutputCurrent());
				SmartDashboard.putNumber("Drive Left 2 Amps", leftDrive2.getOutputCurrent());
				SmartDashboard.putNumber("Drive Left 3 Amps", leftDrive3.getOutputCurrent());
				// SmartDashboard.putNumber("Drive Left Average Amps", getAverageLeftCurrent());
				SmartDashboard.putNumber("Drive Right 1 Amps", rightDrive1.getOutputCurrent());
				SmartDashboard.putNumber("Drive Right 2 Amps", rightDrive2.getOutputCurrent());
				SmartDashboard.putNumber("Drive Right 3 Amps", rightDrive3.getOutputCurrent());
				// SmartDashboard.putNumber("Drive Right Average Amps",
				// getAverageRightCurrent());
				SmartDashboard.putNumber("Yaw Angle Deg", getGyroAngleDeg());
				SmartDashboard.putNumber("Pitch Angle Deg", getGyroPitchAngle());
				SmartDashboard.putData("Diff Drive", m_drive);
				NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
				NetworkTableEntry tx = table.getEntry("tx");
				NetworkTableEntry ty = table.getEntry("ty");
				NetworkTableEntry ta = table.getEntry("ta");
				SmartDashboard.putNumber("Limelight Valid", table.getEntry("tv").getDouble(0));
				SmartDashboard.putNumber("Limelight X", table.getEntry("tx").getDouble(0));
				SmartDashboard.putNumber("Limelight Y", table.getEntry("ty").getDouble(0));
				SmartDashboard.putNumber("Limelight Area", table.getEntry("ta").getDouble(0));

				SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
				SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
				SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
				SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
				SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
				SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

				SmartDashboard.putNumber("x err", mPeriodicIO.error.getTranslation().x());
				SmartDashboard.putNumber("y err", mPeriodicIO.error.getTranslation().y());
				SmartDashboard.putNumber("theta err", mPeriodicIO.error.getRotation().getDegrees());
			} catch (Exception e) {
			}
		} else if (operationMode == Robot.OperationMode.COMPETITION) {
			SmartDashboard.putBoolean("Vison = ", isValid());
			if (getHeading() != null) {
				// SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
			}
		}
	}

	public boolean checkSystem() {
		boolean leftSide = TalonSRXChecker.CheckTalons(this, new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
			{
				add(new TalonSRXChecker.TalonSRXConfig("left_master", leftDrive1));
				add(new TalonSRXChecker.TalonSRXConfig("left_slave", leftDrive2));
				add(new TalonSRXChecker.TalonSRXConfig("left_slave1", leftDrive3));
			}
		}, new TalonSRXChecker.CheckerConfig() {
			{
				mCurrentFloor = 2;
				mRPMFloor = 1500;
				mCurrentEpsilon = 2.0;
				mRPMEpsilon = 250;
				mRPMSupplier = () -> leftDrive1.getSelectedSensorVelocity(0);
			}
		});

		boolean rightSide = TalonSRXChecker.CheckTalons(this, new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
			{
				add(new TalonSRXChecker.TalonSRXConfig("right_master", rightDrive1));
				add(new TalonSRXChecker.TalonSRXConfig("right_slave", rightDrive2));
				add(new TalonSRXChecker.TalonSRXConfig("right_slave1", rightDrive3));
			}
		}, new TalonSRXChecker.CheckerConfig() {

			{
				mCurrentFloor = 2;
				mRPMFloor = 1500;
				mCurrentEpsilon = 2.0;
				mRPMEpsilon = 250;
				mRPMSupplier = () -> rightDrive1.getSelectedSensorVelocity(0);
			}
		});
		return leftSide && rightSide;

	}

	public static void main(String[] args) {

		Drive drive = Drive.getInstance();

		for (int i = 0; i < 10; i++) {
			double m_steerOutput = drive.adjustForSensitivity(1.0, 0, (double) i / 10.0, -3, STEER_NON_LINEARITY);
			System.out.println("i=" + i + ", output=" + m_steerOutput);
		}
	}

}