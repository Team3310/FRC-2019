package frc.team3310.robot.subsystems;

import java.nio.file.Path;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3310.robot.Constants;
import frc.team3310.robot.OI;
import frc.team3310.robot.Robot;
import frc.team3310.robot.RobotMap;
import frc.team3310.robot.loops.Loop;
import frc.team3310.utility.BHRDifferentialDrive;
import frc.team3310.utility.BHRMathUtils;
import frc.team3310.utility.DriveMotionPlanner;
import frc.team3310.utility.DriveSignal;
import frc.team3310.utility.MPSoftwarePIDController;
import frc.team3310.utility.MPSoftwarePIDController.MPSoftwareTurnType;
import frc.team3310.utility.MPTalonPIDController;
import frc.team3310.utility.PIDParams;
import frc.team3310.utility.ReflectingCSVWriter;
import frc.team3310.utility.SoftwarePIDController;
import frc.team3310.utility.lib.control.RobotStatus;
import frc.team3310.utility.lib.drivers.TalonSRXChecker;
import frc.team3310.utility.lib.drivers.TalonSRXEncoder;
import frc.team3310.utility.lib.drivers.TalonSRXFactory;
import frc.team3310.utility.lib.geometry.Pose2d;
import frc.team3310.utility.lib.geometry.Pose2dWithCurvature;
import frc.team3310.utility.lib.geometry.Rotation2d;
import frc.team3310.utility.lib.trajectory.TrajectoryIterator;
import frc.team3310.utility.lib.trajectory.timing.TimedState;

public class Drive extends Subsystem implements Loop {
	private static Drive instance;

	public static enum DriveControlMode { // REMOVED ADAPTIVE_PURSUIT added PATH_FOLLOWING and OPEN_LOOP Poofs
		JOYSTICK, MP_STRAIGHT, MP_TURN, PID_TURN, HOLD, MANUAL, VELOCITY_SETPOINT, CAMERA_TRACK, PATH_FOLLOWING,
		OPEN_LOOP,CAMERA_TRACK_DRIVE
	};

	public static enum DriveSpeedShiftState {
		HI, LO
	};

	public static enum ClimberState {
		DEPLOYED, RETRACTED
	};

	// One revolution of the wheel = Pi * D inches = 60/24 revs due to gears * 36/12
	// revs due mag encoder gear on ball shifter * 4096 ticks
	public static final double ENCODER_TICKS_TO_INCHES = (36.0 / 12.0) * (60.0 / 24.0) * 4096.0 / (5.8 * Math.PI);
	private static final double DRIVE_ENCODER_PPR = 4096.; // Poofs Equvialant to ENCODER_TICKS_TO_INCHES??
	public static final double TRACK_WIDTH_INCHES = 24.56; // 26.937;

	// Motion profile max velocities and accel times
	public static final double MAX_TURN_RATE_DEG_PER_SEC = 320;
	public static final double MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC = 140; // 72;
	public static final double MP_AUTON_MAX_LO_GEAR_STRAIGHT_VELOCITY_INCHES_PER_SEC = 320;
	public static final double MP_AUTON_MAX_HIGH_GEAR_STRAIGHT_VELOCITY_INCHES_PER_SEC = 400;
	public static final double MP_AUTON_MAX_TURN_RATE_DEG_PER_SEC = 270;
	public static final double MP_SLOW_VELOCITY_INCHES_PER_SEC = 25;
	public static final double MP_SLOW_MEDIUM_VELOCITY_INCHES_PER_SEC = 50;
	public static final double MP_MEDIUM_VELOCITY_INCHES_PER_SEC = 80;
	public static final double MP_FAST_VELOCITY_INCHES_PER_SEC = 100;

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

	private BHRDifferentialDrive m_drive;
	
	private boolean isRed = true;
	private boolean mIsBrakeMode;
	private boolean mIsHighGear;

	
	private long periodMs = (long)(Constants.kLooperDt * 1000.0);
	
    protected Rotation2d mAngleAdjustment = Rotation2d.identity();

	// Pneumatics
	private Solenoid speedShift;
	private DriveSpeedShiftState shiftState = DriveSpeedShiftState.HI;

	// Input devices
	public static final int DRIVER_INPUT_JOYSTICK_ARCADE = 0;
	public static final int DRIVER_INPUT_JOYSTICK_TANK = 1;
	public static final int DRIVER_INPUT_JOYSTICK_CHEESY = 2;
	public static final int DRIVER_INPUT_XBOX_CHEESY = 3;
	public static final int DRIVER_INPUT_XBOX_ARCADE_LEFT = 4;
	public static final int DRIVER_INPUT_XBOX_ARCADE_RIGHT = 5;
	public static final int DRIVER_INPUT_WHEEL = 6;

	public static final double STEER_NON_LINEARITY = 0.5;
	public static final double MOVE_NON_LINEARITY = 1.0;
	
	public static final double STICK_DEADBAND = 0.02;
	
//	public static final double PITCH_THRESHOLD_1 = 20;
	public static final double PITCH_THRESHOLD_2 = 25;
	
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
	
    private static final int kLowGearPositionControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;
    private static final int kLowGearVelocityControlSlot = 2;

    private MPTalonPIDController mpStraightController;
//	private PIDParams mpStraightPIDParams = new PIDParams(0.1, 0, 0, 0.005, 0.03, 0.15);  // 4 colsons
	private PIDParams mpStraightPIDParams = new PIDParams(0.05, 0, 0, 0.0008, 0.004, 0.03);  // 4 omni
	private PIDParams mpHoldPIDParams = new PIDParams(1, 0, 0, 0.0, 0.0, 0.0); 

	private MPSoftwarePIDController mpTurnController; // p    i   d     a      v      g    izone
//	private PIDParams mpTurnPIDParams = new PIDParams(0.07, 0.00002, 0.5, 0.00025, 0.008, 0.0, 100);  // 4 colson wheels
	private PIDParams mpTurnPIDParams = new PIDParams(0.03, 0.00002, 0.4, 0.0004, 0.0030, 0.0, 100);  // 4 omni
	
	private SoftwarePIDController pidTurnController;
	private PIDParams pidTurnPIDParams = new PIDParams(0.04, 0.001, 0.4, 0, 0, 0.0, 100); //i=0.0008

//	private PIDParams adaptivePursuitPIDParams = new PIDParams(0.1, 0.00, 1, 0.44); 
// private PathFollower mPathFollower;	REMOVED PathFollower class Poofs
    private Path mCurrentPath = null;
	
	private PigeonIMU gyroPigeon;
	private double[] yprPigeon = new double[3];
	private boolean useGyroLock;
	private double gyroLockAngleDeg;
	private double kPGyro = 0.04;
	private boolean isCalibrating = false;
	private double gyroOffsetDeg = 0;
	
	private double mLastValidGyroAngle;
	private double mCameraVelocity;
	private double kCamera = 0.8;
	private double kCameraDrive = 0.05;
	
	private double limeArea;
	private double limeX;
	private double limeY;
	private double limeSkew;
	private boolean isLimeValid;
	private double LEDMode;
	private double camMode;

	 // Hardware states //Poofs
	 private PeriodicIO mPeriodicIO;
	 private boolean mAutoShift;
	 private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
	 private DriveMotionPlanner mMotionPlanner;
	 private Rotation2d mGyroOffset = Rotation2d.identity();
	 private boolean mOverrideTrajectory = false;

	 
	
    /**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(DriveControlMode state) {
        if (state == DriveControlMode.VELOCITY_SETPOINT || state == DriveControlMode.PATH_FOLLOWING || state == DriveControlMode.CAMERA_TRACK) {
            return true;
        }
        return false;
    }

    /**
     * Check if the drive talons are configured for position control
     */
    protected static boolean usesTalonPositionControl(DriveControlMode state) {
        if (state == DriveControlMode.MP_STRAIGHT ||
                state == DriveControlMode.MP_TURN ||
                state == DriveControlMode.HOLD) {
            return true;
        }
        return false;
    }

    private Drive() {
		try {
			mPeriodicIO = new PeriodicIO();

			leftDrive1 = TalonSRXFactory.createTalonEncoder(RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID, ENCODER_TICKS_TO_INCHES, false, FeedbackDevice.QuadEncoder);
			leftDrive2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR2_CAN_ID, RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);
			leftDrive3 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR3_CAN_ID, RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);

			rightDrive1 = TalonSRXFactory.createTalonEncoder(RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID, ENCODER_TICKS_TO_INCHES, true, FeedbackDevice.QuadEncoder);
			rightDrive2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR2_CAN_ID, RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);
			rightDrive3 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR3_CAN_ID, RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);
			
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
							
			motorControllers.add(leftDrive1);
			motorControllers.add(rightDrive1);
			
			m_drive = new BHRDifferentialDrive(leftDrive1, rightDrive1);
			m_drive.setSafetyEnabled(false);
			mMotionPlanner = new DriveMotionPlanner();


			gyroPigeon = new PigeonIMU(rightDrive2);
			//gyroPigeon.clearStickyFaults(10);
			
			speedShift = new Solenoid(RobotMap.DRIVETRAIN_SPEEDSHIFT_PCM_ID);
									
			reloadGains();
        	setBrakeMode(true);
	}
		catch (Exception e) {
			System.err.println("An error occurred in the DriveTrain constructor");
		}
	}

    
    private void setOpenLoopVoltageRamp(double timeTo12VSec) {
		leftDrive1.configOpenloopRamp(timeTo12VSec, TalonSRXEncoder.TIMEOUT_MS);
		rightDrive1.configOpenloopRamp(timeTo12VSec, TalonSRXEncoder.TIMEOUT_MS);
    }
//REMOVED New Version Poofs 
/*     public synchronized void loadGains() {
        leftDrive1.setPIDFIZone(kLowGearVelocityControlSlot, 
        		Constants.kDriveLowGearVelocityKp, 
        		Constants.kDriveLowGearVelocityKi,
                Constants.kDriveLowGearVelocityKd, 
                Constants.kDriveLowGearVelocityKf,
                Constants.kDriveLowGearVelocityIZone);
        
        rightDrive1.setPIDFIZone(kLowGearVelocityControlSlot, 
        		Constants.kDriveLowGearVelocityKp, 
        		Constants.kDriveLowGearVelocityKi,
                Constants.kDriveLowGearVelocityKd, 
                Constants.kDriveLowGearVelocityKf,
                Constants.kDriveLowGearVelocityIZone);
        
        leftDrive1.setPIDFIZone(kHighGearVelocityControlSlot, 
        		Constants.kDriveHighGearVelocityKp, 
        		Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, 
                Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone);
        
        rightDrive1.setPIDFIZone(kHighGearVelocityControlSlot, 
        		Constants.kDriveHighGearVelocityKp, 
        		Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, 
                Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone);        
    } */

    @Override
	public void initDefaultCommand() {
	}
	
	public synchronized double getGyroAngleDeg() {
		gyroPigeon.getYawPitchRoll(yprPigeon);
		return -yprPigeon[0] + gyroOffsetDeg;
	}
	
	public synchronized double getGyroPitchAngle() {
		gyroPigeon.getYawPitchRoll(yprPigeon);
		return  yprPigeon[2];
	}
	public boolean checkPitchAngle() {
		double pitchAngle = Math.abs(getGyroPitchAngle());
		if(pitchAngle > 10) {
			return true;
		}
		return false;
	}
	
	public synchronized void resetGyro() {
		gyroPigeon.setYaw(0, TalonSRXEncoder.TIMEOUT_MS);
		gyroPigeon.setFusedHeading(0, TalonSRXEncoder.TIMEOUT_MS);
	}
	
    public synchronized Rotation2d getGyroAngle() {
        return mAngleAdjustment.rotateBy(Rotation2d.fromDegrees(-getGyroAngleDeg()));
    }

    public synchronized void setGyroAngle(Rotation2d adjustment) {
    	resetGyro();
        mAngleAdjustment = adjustment;
    }

	public synchronized void resetEncoders() {
		rightDrive1.setPosition(0);
		leftDrive1.setPosition(0);
	}
	
    public void zeroSensors() {
        resetEncoders();
        resetGyro();
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
		
	public void setStraightMP(double distanceInches, double maxVelocity, boolean useGyroLock, boolean useAbsolute, double desiredAbsoluteAngle) {
		double yawAngle = useAbsolute ? BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), desiredAbsoluteAngle) : getGyroAngleDeg();
		mpStraightController.setPID(mpStraightPIDParams, kLowGearPositionControlSlot);
		mpStraightController.setPIDSlot(kLowGearPositionControlSlot);
		mpStraightController.setMPStraightTarget(0, distanceInches, maxVelocity, MP_STRAIGHT_T1, MP_STRAIGHT_T2, useGyroLock, yawAngle, true); 
		setControlMode(DriveControlMode.MP_STRAIGHT);
	}
		
	public void setRelativeTurnMP(double relativeTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(getGyroAngleDeg(), relativeTurnAngleDeg + getGyroAngleDeg(), turnRateDegPerSec, MP_TURN_T1, MP_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		setControlMode(DriveControlMode.MP_TURN);
	}
		
	public void setRelativeMaxTurnMP(double relativeTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(getGyroAngleDeg(), relativeTurnAngleDeg + getGyroAngleDeg(), turnRateDegPerSec, MP_MAX_TURN_T1, MP_MAX_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		setControlMode(DriveControlMode.MP_TURN);
	}
	
	public void setAbsoluteTurnMP(double absoluteTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(getGyroAngleDeg(), BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), absoluteTurnAngleDeg), turnRateDegPerSec, MP_TURN_T1, MP_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		setControlMode(DriveControlMode.MP_TURN);
	}
	
    public void setDriveHold(boolean status) {
		if (status) {
			setControlMode(DriveControlMode.HOLD);
		}
		else {
			setControlMode(DriveControlMode.JOYSTICK);
		}
	}
    
    public synchronized void setControlMode(DriveControlMode controlMode) {
 		this.driveControlMode = controlMode;
 		if (controlMode == DriveControlMode.HOLD) {
			mpStraightController.setPID(mpHoldPIDParams, kLowGearPositionControlSlot);
			leftDrive1.setPosition(0);
			leftDrive1.set(ControlMode.Position, 0);
			rightDrive1.setPosition(0);
			rightDrive1.set(ControlMode.Position, 0);
		}
		setFinished(false);
	}
    
    public synchronized DriveControlMode getControlMode() {
    	return driveControlMode;
    }
	
	@Override
	public void onStart(double timestamp) {
        synchronized (Drive.this) {
			mpStraightController = new MPTalonPIDController(periodMs, motorControllers);
			mpStraightController.setPID(mpStraightPIDParams, kLowGearPositionControlSlot);
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

			if (currentControlMode == DriveControlMode.JOYSTICK) {
				driveWithJoystick();
			}
			else if (!isFinished()) {
				switch (currentControlMode) {			
					case MP_STRAIGHT :
						setFinished(mpStraightController.controlLoopUpdate(getGyroAngleDeg())); 
	                    break;
					case MP_TURN:
						setFinished(mpTurnController.controlLoopUpdate(getGyroAngleDeg())); 
	                    break;
					case PID_TURN:
						setFinished(pidTurnController.controlLoopUpdate(getGyroAngleDeg())); 
						break;
					case PATH_FOLLOWING: //Removes ADAPTIVE_PURSUIT added PATH_FOLLOWING Poofs
						readPeriodicInputs();
						updatePathFollower(); //ADDED updatePathFollower Method Below
						writePeriodicOutputs();
						break;	                
				 	case CAMERA_TRACK:
	                    updateCameraTrack(); //Removed method not sure if needed
						return;
	                default:
	                    System.out.println("Unknown drive control mode: " + currentControlMode);
	                    break;
                }
			}
			else {
				// hold in current state
			}
		}
	}
	
	public synchronized void setSpeed(double speed) {
		if (speed == 0) {
			setControlMode(DriveControlMode.JOYSTICK);
		}
		else {
			setControlMode(DriveControlMode.MANUAL);
			rightDrive1.set(ControlMode.PercentOutput, speed);
			leftDrive1.set(ControlMode.PercentOutput, speed);
		}
	}
	
	public synchronized void setGyroLock(boolean useGyroLock, boolean snapToAbsolute0or180) {
		if (snapToAbsolute0or180) {
			gyroLockAngleDeg = BHRMathUtils.adjustAccumAngleToClosest180(getGyroAngleDeg());
		}
		else {
			gyroLockAngleDeg = getGyroAngleDeg();
		}
		this.useGyroLock = useGyroLock;
	}

    /**
     * Called periodically when the robot is in cmaera track mode.
     */
	//REMOVED POOFS
     private void updateCameraTrack() {
    	updateLimelight();
    	double deltaVelocity = 0;
		mLastValidGyroAngle = getGyroAngleDeg();
        if (isLimeValid) {
        	deltaVelocity = limeX * kCamera;
            System.out.println("Valid lime angle = " + limeX);
        } else {
        	deltaVelocity = (getGyroAngleDeg() - mLastValidGyroAngle) * kCamera;
            System.out.println("In Valid lime angle = " + limeX);
        }
        updateVelocitySetpoint(mCameraVelocity + deltaVelocity, mCameraVelocity - deltaVelocity);
    } 

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     * 
     * @see Path
     */
	//REMOVED Poofs
     public synchronized void setCameraTrack(double straightVelocity) {
        if (driveControlMode != DriveControlMode.CAMERA_TRACK) {
        	setFinished(false);
            configureTalonsForSpeedControl(); //Our Equivalent to Poofs SetVelocity?
            driveControlMode = DriveControlMode.CAMERA_TRACK;
            mLastValidGyroAngle = getGyroAngleDeg();
            mCameraVelocity = straightVelocity;
        } else {
            setVelocitySetpoint(0, 0);
            System.out.println("Oh NOOOO in velocity set point for camera track");
        }
	} 


	   //OLD updatePathFollower CHANGED Poofs 
/*     private void updatePathFollower(double timestamp) {
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose,
                RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
        if (!mPathFollower.isFinished()) {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updateVelocitySetpoint(setpoint.left, setpoint.right);
        } else {
            updateVelocitySetpoint(0, 0);
        }
    } */

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     * 
     * @see Path
     */

//REMOVED Old Path Following Poofs 
/*     public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || driveControlMode != DriveControlMode.ADAPTIVE_PURSUIT) {
            configureTalonsForSpeedControl();
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(
                            new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                    Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
                            Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                            Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                            Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                            Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
                            Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
                            Constants.kPathStopSteeringDistance));

            driveControlMode = DriveControlMode.ADAPTIVE_PURSUIT;
            mCurrentPath = path;
        } else {
            setVelocitySetpoint(0, 0);
            System.out.println("Oh NOOOO in velocity set point");
        }
    } */

    /**
     * Start up velocity mode. This sets the drive train in high gear as well.
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
	//OLD configure dont know if we need to keep this one Poofs 
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        driveControlMode = DriveControlMode.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    } 

    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
	//OLD configure dont know if we need to keep this one Poofs 
     private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (usesTalonVelocityControl(driveControlMode)) {
            final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double maxSetpoint = getShiftState() == DriveSpeedShiftState.HI ? Constants.kDriveHighGearMaxSetpoint : Constants.kDriveLowGearMaxSetpoint;
            final double scale = max_desired > maxSetpoint ? maxSetpoint / max_desired : 1.0;
            
            leftDrive1.setVelocityWorld(left_inches_per_sec * scale);
            rightDrive1.setVelocityWorld(right_inches_per_sec * scale);
//            double command = leftDrive1.convertEncoderWorldToTicks(left_inches_per_sec * scale) * 0.1;
//            System.out.println("vel Com u/s = " + command + ", vel com in/sec= " + left_inches_per_sec * scale + ", scale = " + scale + ", left pos in = " + getLeftPositionInches()  + ", right pos in = " + getRightPositionInches() + ", left vel in/sec = " + getLeftVelocityInchesPerSec() + ", left vel u/s = " + leftDrive1.getSelectedSensorVelocity(0));
        } else {
            System.out.println("Hit a bad velocity control state");
            leftDrive1.set(ControlMode.Velocity, 0);
            rightDrive1.set(ControlMode.Velocity, 0);
        }
    } 

    /**
     * Configures talons for velocity control
     */
	//OLD configure dont know if we need to keep this one Poofs 
	//Added Back in January 9th, 2019 (Said it was needed by Mr.Selle. Used in setVelocitySetpoint, setWantDrivePath, setCameraTrack,old updatePathFollower)
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
       	
        	if (getShiftState() == DriveSpeedShiftState.HI) {
        		System.out.println("configureTalonsForSpeedControl HI");
	        	leftDrive1.selectProfileSlot(kHighGearVelocityControlSlot, TalonSRXEncoder.PID_IDX);
	        	leftDrive1.configNominalOutputForward(Constants.kDriveHighGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	        	leftDrive1.configNominalOutputReverse(-Constants.kDriveHighGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	            leftDrive1.configClosedloopRamp(Constants.kDriveHighGearVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);
	        	    	
	        	rightDrive1.selectProfileSlot(kHighGearVelocityControlSlot, TalonSRXEncoder.PID_IDX);
	        	rightDrive1.configNominalOutputForward(Constants.kDriveHighGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	        	rightDrive1.configNominalOutputReverse(-Constants.kDriveHighGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	        	rightDrive1.configClosedloopRamp(Constants.kDriveHighGearVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);
        	}
        	else {
        		System.out.println("configureTalonsForSpeedControl LO");
	        	leftDrive1.selectProfileSlot(kLowGearVelocityControlSlot, TalonSRXEncoder.PID_IDX);
	        	leftDrive1.configNominalOutputForward(Constants.kDriveLowGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	        	leftDrive1.configNominalOutputReverse(-Constants.kDriveLowGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	            leftDrive1.configClosedloopRamp(Constants.kDriveLowGearVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);
	        	    	
	        	rightDrive1.selectProfileSlot(kLowGearVelocityControlSlot, TalonSRXEncoder.PID_IDX);
	        	rightDrive1.configNominalOutputForward(Constants.kDriveLowGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	        	rightDrive1.configNominalOutputReverse(-Constants.kDriveLowGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	        	rightDrive1.configClosedloopRamp(Constants.kDriveLowGearVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);
        	}
        }
    }  
//REMOVED Old Path Follower Poofs 
/*     public synchronized boolean isDoneWithPath() {
        if (driveControlMode == DriveControlMode.ADAPTIVE_PURSUIT && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode 1");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (driveControlMode == DriveControlMode.ADAPTIVE_PURSUIT && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode 2, control mode = " + driveControlMode);
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (driveControlMode == DriveControlMode.ADAPTIVE_PURSUIT && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode 3. Control mode = " + driveControlMode);
            return false;
        }
    } */

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

    public synchronized void driveWithJoystick() {
		if(m_drive == null) return;

		boolean cameraTrackTapeButton = OI.getInstance().getDriverController().getRightBumper().get();
		boolean cameraTrackCargoButton = OI.getInstance().getDriverController().getButtonY().get();

		m_moveInput = OI.getInstance().getDriverController().getLeftYAxis();
		m_steerInput = -OI.getInstance().getDriverController().getRightXAxis();
		
		m_moveOutput = adjustForSensitivity(m_moveScale, m_moveTrim,
					m_moveInput, m_moveNonLinear, MOVE_NON_LINEARITY);
		m_steerOutput = adjustForSensitivity(m_steerScale, m_steerTrim,
				m_steerInput, m_steerNonLinear, STEER_NON_LINEARITY);

		if (useGyroLock) {
			double yawError = gyroLockAngleDeg - getGyroAngleDeg();
			m_steerOutput = kPGyro * yawError;
		}
				
		double pitchAngle = updatePitchWindow();
		if(Math.abs(pitchAngle) > PITCH_THRESHOLD_2) {
			m_moveOutput = Math.signum(pitchAngle) * -1.0;
			m_steerOutput = 0;
			System.out.println("Pitch Treshhold 2 angle = " + pitchAngle);
		}

		if(cameraTrackTapeButton){
			updateLimelight();
			double cameraSteer = 0;
			mLastValidGyroAngle = getGyroAngleDeg();
			if (isLimeValid) {
				cameraSteer = limeX * kCameraDrive;
				System.out.println("Valid lime angle = " + limeX);
			} 
			else {
				//cameraSteer = (getGyroAngleDeg() - mLastValidGyroAngle) * kCameraDrive;
				System.out.println("In Valid lime angle = " + limeX);
				cameraSteer = -m_steerOutput;
			}
			m_steerOutput = -cameraSteer;
		}

		if(cameraTrackCargoButton){
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("").setNumber(5);
			updateLimelight();
			double cameraSteer = 0;
			mLastValidGyroAngle = getGyroAngleDeg();
			if (isLimeValid) {
				cameraSteer = limeX * kCameraDrive;
				System.out.println("CARGO FOUND = " + limeX);
			} 
			else {
				//cameraSteer = (getGyroAngleDeg() - mLastValidGyroAngle) * kCameraDrive;
				System.out.println("NO CARGO FOUND = " + limeX);
				cameraSteer = -m_steerOutput;
			}
			m_steerOutput = -cameraSteer;
		}

		m_drive.arcadeDrive(-m_moveOutput, -m_steerOutput);	
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
		
    	return pitchSum/pitchWindowSize;
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

	public double adjustForSensitivity(double scale, double trim,
			double steer, int nonLinearFactor, double wheelNonLinearity) {
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

	private double nonlinearStickCalcPositive(double steer,
			double steerNonLinearity) {
		return Math.sin(Math.PI / 2.0 * steerNonLinearity * steer)
				/ Math.sin(Math.PI / 2.0 * steerNonLinearity);
	}

	private double nonlinearStickCalcNegative(double steer,
			double steerNonLinearity) {
		return Math.asin(steerNonLinearity * steer)
				/ Math.asin(steerNonLinearity);
	}

	public void setShiftState(DriveSpeedShiftState state) {
		shiftState = state;

		System.out.println("shift state = " + state);
		setOpenLoopVoltageRamp(state == DriveSpeedShiftState.HI ? OPEN_LOOP_VOLTAGE_RAMP_HI : OPEN_LOOP_VOLTAGE_RAMP_LO);
		if(state == DriveSpeedShiftState.HI) {
			speedShift.set(false);
		}
		else if(state == DriveSpeedShiftState.LO) {
			speedShift.set(true);
		}
	}

	public DriveSpeedShiftState getShiftState() {
		return shiftState;
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
	
	public boolean isRed() {
		return isRed;
	}
	
	public void setIsRed(boolean status) {
		isRed = status;
	}
	
	public static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    public static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }
    
    public double getRightPositionInches() {
    	return rightDrive1.getPositionWorld();
    }

    public double getLeftPositionInches() {
    	return leftDrive1.getPositionWorld();
    }

    public double getRightVelocityInchesPerSec() {
    	return rightDrive1.getVelocityWorld();
    }

    public double getLeftVelocityInchesPerSec() {
    	return leftDrive1.getVelocityWorld();
    }
    
    public double getAverageLeftCurrent() {
    	return (leftDrive1.getOutputCurrent() + leftDrive2.getOutputCurrent() + leftDrive3.getOutputCurrent()) / 3;
    }

    public double getAverageRightCurrent() {
    	return (rightDrive1.getOutputCurrent() + rightDrive2.getOutputCurrent() + rightDrive3.getOutputCurrent()) / 3;
    }
    
	public NetworkTable getLimetable() {
		return NetworkTableInstance.getDefault().getTable("limelight");
	}

	private void updateLimelight() {
		NetworkTable limeTable = getLimetable();		
		double valid = limeTable.getEntry("tv").getDouble(0); 
		if (valid == 0) {
			isLimeValid = false;
		}
		else if (valid == 1) {
			isLimeValid = true;
		}
		
		limeX = limeTable.getEntry("tx").getDouble(0); 
		limeY = limeTable.getEntry("ty").getDouble(0); 
		limeArea = limeTable.getEntry("ta").getDouble(0); 
		limeSkew = limeTable.getEntry("ts").getDouble(0); 
	}
	
	//Set the LED mode of the limelight
	public void setLimeLED(boolean isOn) {
		getLimetable().getEntry("ledMode").setDouble(isOn ? 0 : 1);
	}
	
	//Set the camera mode
	public void setLimeCameraMode(boolean isOn) {
		getLimetable().getEntry("camMode").setDouble(isOn ? 1 : 0);
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
				SmartDashboard.putNumber("Drive Left Average Amps", getAverageLeftCurrent());
				SmartDashboard.putNumber("Drive Right 1 Amps", rightDrive1.getOutputCurrent());
				SmartDashboard.putNumber("Drive Right 2 Amps", rightDrive2.getOutputCurrent());
				SmartDashboard.putNumber("Drive Right 3 Amps", rightDrive3.getOutputCurrent());
				SmartDashboard.putNumber("Drive Right Average Amps", getAverageRightCurrent());
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
			}
			catch (Exception e) {
			}
		}
	}	
	
	public static Drive getInstance() {
		if(instance == null) {
			instance = new Drive();
		}
		return instance;
	}


//ALL NEW POOFS MOTOR+ SETUP 

private static double radiansPerSecondToTicksPer100ms(double rad_s) {
	return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
}

public double getLeftEncoderRotations() {
	return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR; 
}

public double getRightEncoderRotations() {
	return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
}

public double getLeftEncoderDistance() {
	return rotationsToInches(getLeftEncoderRotations());
}

public double getRightEncoderDistance() {
	return rotationsToInches(getRightEncoderRotations());
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

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (driveControlMode != DriveControlMode.OPEN_LOOP) {
            setBrakeMode(false);
            mAutoShift = true;

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
        if (driveControlMode != DriveControlMode.PATH_FOLLOWING) {
            // We entered a velocity control state.
            setBrakeMode(true);
            mAutoShift = false;
            leftDrive1.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            rightDrive1.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            leftDrive1.configNeutralDeadband(0.0, 0);
            rightDrive1.configNeutralDeadband(0.0, 0);

            driveControlMode = DriveControlMode.PATH_FOLLOWING;
        }
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
            driveControlMode = DriveControlMode.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || driveControlMode != DriveControlMode.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            speedShift.set(wantsHighGear);
        }
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
//ALL NEW POOFS PATH FOLLOWING 
public void overrideTrajectory(boolean value) {
	mOverrideTrajectory = value;
}

private void updatePathFollower() {
	if (driveControlMode == DriveControlMode.PATH_FOLLOWING) {
		final double now = Timer.getFPGATimestamp();

		DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotStatus.getInstance().getFieldToVehicle(now));

		// DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

		mPeriodicIO.error = mMotionPlanner.error();
		mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

		if (!mOverrideTrajectory) {
			setVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
					new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

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
//I dont think we need this but if so not difficult to add Poofs
 private void handleAutoShift() {
	final double linear_velocity = Math.abs(getLinearVelocity());
	final double angular_velocity = Math.abs(getAngularVelocity());
	if (mIsHighGear && linear_velocity < Constants.kDriveDownShiftVelocity && angular_velocity < Constants
			.kDriveDownShiftAngularVelocity) {
		setHighGear(false);
	} else if (!mIsHighGear && linear_velocity > Constants.kDriveUpShiftVelocity) {
		setHighGear(true);
	}
}

public synchronized void reloadGains() {
	leftDrive1.config_kP(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
	leftDrive1.config_kI(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
	leftDrive1.config_kD(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
	leftDrive1.config_kF(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
	leftDrive1.config_IntegralZone(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);

	rightDrive1.config_kP(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
	rightDrive1.config_kI(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
	rightDrive1.config_kD(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
	rightDrive1.config_kF(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
	rightDrive1.config_IntegralZone(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);
	
	//High Gear 
	//ADDED HIGH GEAR POOFS
	leftDrive1.config_kP(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKp, Constants.kLongCANTimeoutMs);
	leftDrive1.config_kI(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKi, Constants.kLongCANTimeoutMs);
	leftDrive1.config_kD(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKd, Constants.kLongCANTimeoutMs);
	leftDrive1.config_kF(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKf, Constants.kLongCANTimeoutMs);
	leftDrive1.config_IntegralZone(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityIZone, Constants.kLongCANTimeoutMs);

	rightDrive1.config_kP(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKp, Constants.kLongCANTimeoutMs);
	rightDrive1.config_kI(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKi, Constants.kLongCANTimeoutMs);
	rightDrive1.config_kD(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKd, Constants.kLongCANTimeoutMs);
	rightDrive1.config_kF(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityKf, Constants.kLongCANTimeoutMs);
	rightDrive1.config_IntegralZone(kHighGearVelocityControlSlot, Constants.kDriveHighGearVelocityIZone, Constants.kLongCANTimeoutMs);
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

	// System.out.println("control state: " + mDriveControlState + ", left: " + mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
}

public synchronized void writePeriodicOutputs() {
	if (driveControlMode == DriveControlMode.OPEN_LOOP) {
		leftDrive1.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
		rightDrive1.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
	} else {
		leftDrive1.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
				mPeriodicIO.left_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / 1023.0);
		rightDrive1.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
				mPeriodicIO.right_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / 1023.0);
	}
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
	public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity()); 
	}
	public boolean checkSystem() {
        boolean leftSide = TalonSRXChecker.CheckTalons(this,
                new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
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

        boolean rightSide = TalonSRXChecker.CheckTalons(this,
                new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
                    {
                        add(new TalonSRXChecker.TalonSRXConfig("right_master", rightDrive1));
                        add(new TalonSRXChecker.TalonSRXConfig("right_slave", rightDrive2));
                        add(new TalonSRXChecker.TalonSRXConfig("right_slave1", rightDrive3));
                    }
               }, new TalonSRXChecker.CheckerConfig() {

                    {                        mCurrentFloor = 2;
                        mRPMFloor = 1500;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 250;
                        mRPMSupplier = () -> rightDrive1.getSelectedSensorVelocity(0);
                   }
              });
        return leftSide && rightSide;

	}
}