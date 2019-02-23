package frc.team3310.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.team3310.robot.Constants;
import frc.team3310.robot.Robot;
import frc.team3310.utility.Util;
import frc.team3310.robot.RobotMap;
import frc.team3310.robot.loops.Loop;
import frc.team3310.utility.MPTalonPIDController;
import frc.team3310.utility.PIDParams;
import frc.team3310.utility.lib.drivers.TalonSRXEncoder;
import frc.team3310.utility.lib.drivers.TalonSRXFactory;
import frc.team3310.utility.lib.drivers.TalonSRXUtil;

public class ElevatorSave20190223 extends Subsystem implements Loop {
	private static ElevatorSave20190223 instance;

	public static enum ElevatorControlMode {
		MOTION_PROFILE, MOTION_MAGIC, JOYSTICK_PID, JOYSTICK_MANUAL, MANUAL
	};

	public static enum ElevatorSpeedShiftState {
		HI, LO
	};

	public static enum FrontLegShiftState {
		IN, OUT
	}

	public static enum BackLegShiftState {
		IN, OUT
	}

	public static enum ElevatorClimbShiftState {
		IN, OUT
	}

	// One revolution of the 30T Drive 1.880" PD pulley = Pi * PD inches = 36/24
	// revs due to pulleys * 34/24 revs due to gears * 36/12 revs due mag encoder
	// gear on ball shifter * 4096 ticks
	// public static final double ENCODER_TICKS_TO_INCHES = (36.0 / 12.0) * (36.0 /
	// 24.0) * (34.0 / 24.0) * 4096.0
	// / (1.88 * Math.PI);

	public static final double ENCODER_TICKS_TO_INCHES_ELEVATOR = (50.0 / 50.0) * (34.0 / 34.0) * 4096.0
			/ (1.2987013 * Math.PI);

	public static final double ENCODER_TICKS_TO_INCHES_GGG = (50.0 / 50.0) * (50.0 / 18.0) * (40.0 / 24.0) * 4096.0
			/ (1.128 * Math.PI);

	// Defined speeds
	public static final double TEST_SPEED_UP = 0.3;
	public static final double TEST_SPEED_DOWN = -0.3;
	public static final double AUTO_ZERO_SPEED = -0.3;
	public static final double JOYSTICK_INCHES_PER_MS_ELEVATOR = 0.75;
	public static final double JOYSTICK_INCHES_PER_MS_GGG = ENCODER_TICKS_TO_INCHES_ELEVATOR
			/ ENCODER_TICKS_TO_INCHES_GGG * 0.8;

	// Motion profile max velocities and accel times
	public static final int MP_MAX_VELOCITY_INCHES_PER_SEC = 60;
	public static final int MP_T1 = 400; // Fast = 300
	public static final int MP_T2 = 150; // Fast = 150

	// Motor controllers
	private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();

	private TalonSRXEncoder motor1;
	private TalonSRX motor2;
	private TalonSRX motor3;

	// PID controller and params
	private MPTalonPIDController mpController;

	public static int PID_SLOT = 0;
	public static int MP_SLOT = 1;

	private PIDParams mpPIDParams = new PIDParams(0.2, 0.0, 0.0, 0.0, 0.005, 0.0);
	private PIDParams pidPIDParamsHiGear = new PIDParams(0.075, 0.0, 0.0, 0.0, 0.0, 0.0);
	public static final double KF_UP = 0.005;
	public static final double KF_DOWN = 0.0;
	public static final double PID_ERROR_INCHES = 1.0;
	private long periodMs = (long) (Constants.kLooperDt * 1000.0);

	// Pneumatics
	// private Solenoid speedShift;
	private Solenoid elevatorShift;
	private Solenoid frontLegShift;
	private Solenoid backLegShift;

	// Motion Magic
	private static final int kMotionMagicSlot = 2;
	private Intake mIntake = Intake.getInstance();
	private PeriodicIO mPeriodicIO = new PeriodicIO();
	private int kHomePositionInches = 0;

	// Shifting Default
	private FrontLegShiftState frontShiftState = FrontLegShiftState.IN; // OUT = Climb Mode
	private BackLegShiftState backShiftState = BackLegShiftState.IN; // OUT = Climb Mode
	private ElevatorClimbShiftState elevatorClimbShiftState = ElevatorClimbShiftState.OUT; // IN = Climb Mode

	// Misc
	public static final double AUTO_ZERO_MOTOR_CURRENT = 4.0;
	private boolean isFinished;
	private ElevatorControlMode elevatorControlMode = ElevatorControlMode.JOYSTICK_MANUAL;
	private double targetPositionInchesPID = 0;
	private boolean firstMpPoint;
	private double joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_ELEVATOR;
	private double currentEncoderTicksToInches = ENCODER_TICKS_TO_INCHES_ELEVATOR;
	public boolean toLow;
	public boolean elevatorCargoMode = false;

	private ElevatorSave20190223() {
		try {
			motor1 = TalonSRXFactory.createTalonEncoder(RobotMap.ELEVATOR_MOTOR_1_CAN_ID, currentEncoderTicksToInches,
					false, FeedbackDevice.QuadEncoder);
			motor2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.ELEVATOR_MOTOR_2_CAN_ID,
					RobotMap.ELEVATOR_MOTOR_1_CAN_ID);
			motor3 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.ELEVATOR_MOTOR_3_CAN_ID,
					RobotMap.ELEVATOR_MOTOR_1_CAN_ID);

			configureTalonsForMotionMagic();

			motor1.setSensorPhase(false);
			motor2.setSensorPhase(false);
			motor3.setSensorPhase(false);
			motor1.setInverted(true);
			motor2.setInverted(true);
			motor3.setInverted(true);

			motorControllers.add(motor1);

			elevatorShift = new Solenoid(RobotMap.ELEVATOR_CLIMB_SHIFT_PCM_ID);
			frontLegShift = new Solenoid(RobotMap.FRONT_LEG_SHIFT_PCM_ID);
			backLegShift = new Solenoid(RobotMap.BACK_LEG_SHIFT_PCM_ID);
		} catch (Exception e) {
			System.err.println("An error occurred in the Elevator constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}

	/**
	 * Configures talons for motion magic
	 */

	public void configureTalonsForMotionMagic() {
		motor1.configFactoryDefault();
		motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
		//		"Could not detect elevator encoder: ");

		// TalonSRXUtil.checkError(
		// 		motor1.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
		// 				LimitSwitchNormal.NormallyOpen, Constants.kLongCANTimeoutMs),
		// 		"Could not set forward (down) limit switch elevator: ");

		// TalonSRXUtil.checkError(
		// 		motor1.configForwardSoftLimitThreshold(Constants.MAX_POSITION_INCHES, Constants.kLongCANTimeoutMs),
		// 		"Could not set forward (down) soft limit switch elevator: ");

		// TalonSRXUtil.checkError(motor1.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs),
		// 		"Could not enable forward (down) soft limit switch elevator: ");

	//	motor1.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
			//	"Could not set voltage compensation saturation elevator: ");

		// TalonSRXUtil.checkError(
		// 		motor1.configReverseSoftLimitThreshold(Constants.MIN_POSITION_INCHES, Constants.kLongCANTimeoutMs),
		// 		"Could not set reverse (up) soft limit switch elevator: ");

		// TalonSRXUtil.checkError(motor1.configReverseSoftLimitEnable(true, Constants.kLongCANTimeoutMs),
		// 		"Could not enable reverse (up) soft limit switch elevator: ");

		// configure magic motion
		motor1.config_kP(kMotionMagicSlot, Constants.kElevatorMotionMagicKp, Constants.kLongCANTimeoutMs);//"Could not set elevator kp: ");

		motor1.config_kI(kMotionMagicSlot, Constants.kElevatorMotionMagicKi, Constants.kLongCANTimeoutMs);
			//	"Could not set elevator ki: ");

		motor1.config_kD(kMotionMagicSlot, Constants.kElevatorMotionMagicKd + Constants.kElevatorMotionMagicKd / 100.0, Constants.kLongCANTimeoutMs);//, "Could not set elevator kd: ");

		motor1.config_kF(kMotionMagicSlot, Constants.kElevatorMotionMagicKf, Constants.kLongCANTimeoutMs);
			//	"Could not set elevator kf: ");

		motor1.configMotionSCurveStrength(Constants.kSmoothing, Constants.kLongCANTimeoutMs);
			//	"Could not set elevator kSmoothing: ");

		// TalonSRXUtil.checkError(motor1.configMaxIntegralAccumulator(kHighGearSlot,
		// 		Constants.kElevatorMaxIntegralAccumulator, Constants.kLongCANTimeoutMs),
		// 		"Could not set elevator max integral: ");

		motor1.config_IntegralZone(kMotionMagicSlot, Constants.kElevatorIZone, Constants.kLongCANTimeoutMs);
			//	"Could not set elevator i zone: ");

		// TalonSRXUtil.checkError(motor1.configAllowableClosedloopError(kHighGearSlot, Constants.kElevatorDeadband,
		// 		Constants.kLongCANTimeoutMs), "Could not set elevator deadband: ");

		motor1.configMotionAcceleration(Constants.kElevatorAcceleration, Constants.kLongCANTimeoutMs);  
		//		"Could not set elevator acceleration: ");

		motor1.configMotionCruiseVelocity(Constants.kElevatorCruiseVelocity, Constants.kLongCANTimeoutMs);
		//		"Could not set elevator cruise velocity: ");

	//	motor1.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
	//	motor1.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);

		motor1.selectProfileSlot(0, 0);

	//	motor1.overrideLimitSwitchesEnable(true);
	//	motor1.overrideSoftLimitsEnable(false);

		motor1.enableVoltageCompensation(true);

	//	motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
	//	motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
	//	motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
	//	motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		motor1.configNominalOutputForward(0.05, Constants.kLongCANTimeoutMs);
		motor1.configNominalOutputReverse(-0.05, Constants.kLongCANTimeoutMs);
		motor1.configPeakOutputForward(1.0, Constants.kLongCANTimeoutMs);
		motor1.configPeakOutputReverse(-1.0, Constants.kLongCANTimeoutMs);
	}

	public void resetZeroPosition(double position) {
		mpController.resetZeroPosition(position);
	}

	public synchronized void resetEncoders() {
		motor1.setPosition(0);
	}

	private synchronized void setElevatorControlMode(ElevatorControlMode controlMode) {
		this.elevatorControlMode = controlMode;
	}

	private synchronized ElevatorControlMode getElevatorControlMode() {
		return this.elevatorControlMode;
	}

	public void setSpeed(double speed) {
		motor1.set(ControlMode.PercentOutput, speed);
		setElevatorControlMode(ElevatorControlMode.MANUAL);
	}

	public void setSpeedJoystick(double speed) {
		motor1.set(ControlMode.PercentOutput, speed);
		setElevatorControlMode(ElevatorControlMode.JOYSTICK_MANUAL);
	}

	public void setPositionPID(double targetPositionInches) {
		mpController.setPIDSlot(PID_SLOT);
		updatePositionPID(targetPositionInches);
		setElevatorControlMode(ElevatorControlMode.JOYSTICK_PID);
		setFinished(false);
	}

	public void updatePositionPID(double targetPositionInches) {
		targetPositionInchesPID = limitPosition(targetPositionInches);
		double startPositionInches = motor1.getPositionWorld();
		mpController.setTarget(targetPositionInchesPID,
				targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN);
	}

	public void setPositionMP(double targetPositionInches) {
		double startPositionInches = motor1.getPositionWorld();
		mpController.setMPTarget(startPositionInches, limitPosition(targetPositionInches),
				MP_MAX_VELOCITY_INCHES_PER_SEC, MP_T1, MP_T2);
		setFinished(false);
		firstMpPoint = true;
		setElevatorControlMode(ElevatorControlMode.MOTION_PROFILE);
	}

	private double limitPosition(double targetPosition) {
		if (targetPosition < Constants.MIN_POSITION_INCHES) {
			return Constants.MIN_POSITION_INCHES;
		} else if (targetPosition > Constants.MAX_POSITION_INCHES) {
			return Constants.MAX_POSITION_INCHES;
		}

		return targetPosition;
	}

	@Override
	public void onStart(double timestamp) {
		mpController = new MPTalonPIDController(periodMs, motorControllers);
		mpController.setPID(mpPIDParams, MP_SLOT);
		mpController.setPID(pidPIDParamsHiGear, PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
		mpController.setPID(pidPIDParamsHiGear, PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
	}

	@Override
	public void onStop(double timestamp) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onLoop(double timestamp) {
		synchronized (ElevatorSave20190223.this) {

	//		readPeriodicInputs();
			switch (getElevatorControlMode()) {
			case JOYSTICK_PID:
				controlPidWithJoystick();
				break;
			case JOYSTICK_MANUAL:
				controlManualWithJoystick();
				break;
			case MOTION_MAGIC:
	//			writePeriodicOutputs();
				break;
			case MOTION_PROFILE:
				if (!isFinished()) {
					if (firstMpPoint) {
						mpController.setPIDSlot(MP_SLOT);
						firstMpPoint = false;
					}
					setFinished(mpController.controlLoopUpdate());
					if (isFinished()) {
						mpController.setPIDSlot(PID_SLOT);
					}
				}
				break;
			default:
				break;
			}
		}
	}

	private void controlPidWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		double deltaPosition = joystickPosition * joystickInchesPerMs;
		targetPositionInchesPID = targetPositionInchesPID + deltaPosition;
		updatePositionPID(targetPositionInchesPID);
	}

	private void controlManualWithJoystick() {
		double joyStickSpeed = -Robot.oi.getOperatorController().getLeftYAxis();
		setSpeedJoystick(joyStickSpeed);
	}

	public synchronized void setMotionMagicPosition(double positionInchesOffGround) {
		double positionInchesFromHome = positionInchesOffGround - kHomePositionInches;
		double encoderPosition = positionInchesFromHome * currentEncoderTicksToInches;
		setElevatorControlMode(ElevatorControlMode.MOTION_MAGIC);
		setClosedLoopRawPosition(encoderPosition);
	}

	private synchronized void setClosedLoopRawPosition(double encoderPosition) {
	//	if (elevatorControlMode != ElevatorControlMode.MOTION_MAGIC) {
	//		elevatorControlMode = ElevatorControlMode.MOTION_MAGIC;
			motor1.selectProfileSlot(kMotionMagicSlot, 0);
	//	}
		mPeriodicIO.demand = encoderPosition;
		motor1.set(ControlMode.MotionMagic, encoderPosition, DemandType.ArbitraryFeedForward, Constants.kElevatorFeedforwardNoBall);
	}

	public synchronized boolean hasFinishedTrajectory() {
		System.out.println("Error=" + motor1.getClosedLoopError());
		return elevatorControlMode == ElevatorControlMode.MOTION_MAGIC
				&& Util.epsilonEquals(mPeriodicIO.active_trajectory_position, mPeriodicIO.demand, 5);
	}

	public synchronized double getRPM() {
		// We are using a CTRE mag encoder which is 4096 native units per revolution.
		return mPeriodicIO.velocity_ticks_per_100ms * 10.0 / 4096.0 * 60.0;
	}

	public synchronized double getInchesOffGround() {
		return (mPeriodicIO.position_ticks / currentEncoderTicksToInches) + kHomePositionInches;
	}

	public synchronized double getSetpoint() {
		return elevatorControlMode == ElevatorControlMode.MOTION_MAGIC
				? mPeriodicIO.demand / currentEncoderTicksToInches + kHomePositionInches
				: Double.NaN;
	}

	public synchronized double getActiveTrajectoryAccelG() {
		return mPeriodicIO.active_trajectory_accel_g;
	}

	public synchronized void readPeriodicInputs() {
		final double t = Timer.getFPGATimestamp();
		mPeriodicIO.position_ticks = motor1.getSelectedSensorPosition(0);
		mPeriodicIO.velocity_ticks_per_100ms = motor1.getSelectedSensorVelocity(0);
		if (motor1.getControlMode() == ControlMode.MotionMagic) {
			mPeriodicIO.active_trajectory_position = motor1.getActiveTrajectoryPosition();
			final int newVel = motor1.getActiveTrajectoryVelocity();
			// TODO check sign of elevator accel
			if (Util.epsilonEquals(newVel, Constants.kElevatorCruiseVelocity, 5)
					|| Util.epsilonEquals(newVel, mPeriodicIO.active_trajectory_velocity, 5)) {
				// Elevator is ~constant velocity.
				mPeriodicIO.active_trajectory_accel_g = 0.0;
			} else if (newVel > mPeriodicIO.active_trajectory_velocity) {
				// Elevator is accelerating downwards.
				mPeriodicIO.active_trajectory_accel_g = -Constants.kElevatorAcceleration * 10.0
						/ (currentEncoderTicksToInches * 386.09);
			} else {
				// Elevator is accelerating upwards.
				mPeriodicIO.active_trajectory_accel_g = Constants.kElevatorAcceleration * 10.0
						/ (currentEncoderTicksToInches * 386.09);
			}
			mPeriodicIO.active_trajectory_velocity = newVel;
		} else {
			mPeriodicIO.active_trajectory_position = Integer.MIN_VALUE;
			mPeriodicIO.active_trajectory_velocity = 0;
			mPeriodicIO.active_trajectory_accel_g = 0.0;
		}
		mPeriodicIO.output_percent = motor1.getMotorOutputPercent();
		mPeriodicIO.limit_switch = motor1.getSensorCollection().isFwdLimitSwitchClosed();
		mPeriodicIO.t = t;

		if (getInchesOffGround() > Constants.kElevatorEpsilon) {
			mPeriodicIO.feedforward = mIntake.hasBall() ? Constants.kElevatorFeedforwardWithBall
					: Constants.kElevatorFeedforwardNoBall;
		} else {
			mPeriodicIO.feedforward = 0.0;
		}
	}

	public synchronized void writePeriodicOutputs() {
		if (elevatorControlMode == ElevatorControlMode.MOTION_MAGIC) {
			motor1.set(ControlMode.MotionMagic, mPeriodicIO.demand, DemandType.ArbitraryFeedForward,
					mPeriodicIO.feedforward);
		} else {
			motor1.set(ControlMode.PercentOutput, mPeriodicIO.demand, DemandType.ArbitraryFeedForward,
					mPeriodicIO.feedforward);
			System.out.print("Not in MM");
		}
	}

	public void setFrontLegState(FrontLegShiftState state) {
		frontShiftState = state;
		if (state == FrontLegShiftState.IN) {
			frontLegShift.set(false);
			System.out.println("FL IN NOT CLIMB");
		} else if (state == FrontLegShiftState.OUT) {
			frontLegShift.set(true);
			System.out.println("FL OUT CLIMB MODE");
		}
	}

	public void setBackLegState(BackLegShiftState state) {
		backShiftState = state;
		if (state == BackLegShiftState.IN) {
			backLegShift.set(false);
			System.out.println("BL IN NOT CLIMB");
		} else if (state == BackLegShiftState.OUT) {
			backLegShift.set(true);
			System.out.println("BL OUT CLIMB MODE");
		}
	}

	public void setElevatorClimbState(ElevatorClimbShiftState state) {
		elevatorClimbShiftState = state;
		if (state == ElevatorClimbShiftState.IN) {
			elevatorShift.set(false);
			System.out.println("GG IN CLIMB");
		} else if (state == ElevatorClimbShiftState.OUT) {
			elevatorShift.set(true);
			System.out.println("GG OUT NOT ClIMb");
		}
	}

	public void setRobotClimbMode() {
		joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_GGG;
		currentEncoderTicksToInches = ENCODER_TICKS_TO_INCHES_GGG;
		setFrontLegState(FrontLegShiftState.OUT);
		setBackLegState(BackLegShiftState.OUT);
		setElevatorClimbState(ElevatorClimbShiftState.IN);
	}

	public void setRobotClimbFront() {
		joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_GGG;
		currentEncoderTicksToInches = ENCODER_TICKS_TO_INCHES_GGG;
		setFrontLegState(FrontLegShiftState.OUT);
		setBackLegState(BackLegShiftState.IN);
		setElevatorClimbState(ElevatorClimbShiftState.IN);
	}

	public void setRobotClimbBack() {
		joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_GGG;
		currentEncoderTicksToInches = ENCODER_TICKS_TO_INCHES_GGG;
		setFrontLegState(FrontLegShiftState.IN);
		setBackLegState(BackLegShiftState.OUT);
		setElevatorClimbState(ElevatorClimbShiftState.IN);
	}

	public void setRobotScoreMode() {
		joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_ELEVATOR;
		currentEncoderTicksToInches = ENCODER_TICKS_TO_INCHES_ELEVATOR;
		setFrontLegState(FrontLegShiftState.IN);
		setBackLegState(BackLegShiftState.IN);
		setElevatorClimbState(ElevatorClimbShiftState.OUT);
	}

	public FrontLegShiftState getFrontShiftState() {
		return frontShiftState;
	}

	public BackLegShiftState getBackShiftState() {
		return backShiftState;
	}

	public ElevatorClimbShiftState getClimbShiftState() {
		return elevatorClimbShiftState;
	}

	public double getPositionInches() {
		return motor1.getPositionWorld();
	}

	public double getAverageMotorCurrent() {
		return (motor1.getOutputCurrent() + motor2.getOutputCurrent() + motor3.getOutputCurrent()) / 3;
	}

	public synchronized boolean isFinished() {
		return isFinished;
	}

	public synchronized void setFinished(boolean isFinished) {
		this.isFinished = isFinished;
	}

	public double getPeriodMs() {
		return periodMs;
	}

	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putNumber("Elevator Position Inches", motor1.getPositionWorld());
				SmartDashboard.putNumber("Elevator Motor 1 Amps", motor1.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Motor 2 Amps", motor2.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Motor 3 Amps", motor3.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Average Amps", getAverageMotorCurrent());
				SmartDashboard.putNumber("Elevator Target PID Position", targetPositionInchesPID);
			} catch (Exception e) {
			}
		} else if (operationMode == Robot.OperationMode.COMPETITION) {
//			SmartDashboard.putNumber("Elevator Sensor Velocity", motor1.getSelectedSensorVelocity());
			SmartDashboard.putNumber("Elevator Position Inches Poofs", getInchesOffGround());
			SmartDashboard.putNumber("Position Ticks Poofs", mPeriodicIO.position_ticks);
			SmartDashboard.putNumber("ClosedLoopTarget", motor1.getClosedLoopTarget());
    		SmartDashboard.putNumber("ActTrajVelocity", motor1.getActiveTrajectoryVelocity());
    		SmartDashboard.putNumber("ActTrajPosition", motor1.getActiveTrajectoryPosition());
			SmartDashboard.putNumber("SensorVel", motor1.getSelectedSensorVelocity());
			SmartDashboard.putNumber("SensorPos", motor1.getSelectedSensorPosition());
			SmartDashboard.putNumber("MotorOutputPercent", motor1.getMotorOutputPercent());
			SmartDashboard.putNumber("ClosedLoopError", motor1.getClosedLoopError());
	
		}
	}

	public static class PeriodicIO {
		// INPUTS
		public int position_ticks;
		public int velocity_ticks_per_100ms;
		public double active_trajectory_accel_g;
		public int active_trajectory_velocity;
		public int active_trajectory_position;
		public double output_percent;
		public boolean limit_switch;
		public double feedforward;
		public double t;

		// OUTPUTS
		public double demand;
	}

	public static ElevatorSave20190223 getInstance() {
		if (instance == null) {
			instance = new ElevatorSave20190223();
		}
		return instance;
	}
}