package frc.team3310.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// USB Port IDs
	public static final int DRIVER_JOYSTICK_1_USB_ID = 0;
	public static final int OPERATOR_JOYSTICK_1_USB_ID = 1;

	// Motors
	public static final int DRIVETRAIN_RIGHT_MOTOR1_CAN_ID = 0;
	public static final int DRIVETRAIN_RIGHT_MOTOR2_CAN_ID = 1;
	public static final int DRIVETRAIN_RIGHT_MOTOR3_CAN_ID = 2;

	public static final int DRIVETRAIN_LEFT_MOTOR1_CAN_ID = 15;
	public static final int DRIVETRAIN_LEFT_MOTOR2_CAN_ID = 14;
	public static final int DRIVETRAIN_LEFT_MOTOR3_CAN_ID = 13;

	public static final int ELEVATOR_MOTOR_1_CAN_ID = 2;
	public static final int ELEVATOR_MOTOR_2_CAN_ID = 4;
	public static final int ELEVATOR_MOTOR_3_CAN_ID = 5;

	public static final int DRIVE_MIDDLE_CLIMB_WHEEL = 12;

	public static final int INTAKE_LEFT_CAN_ID = 10; // 10
	public static final int INTAKE_RIGHT_CAN_ID = 11; // 5

	// Pneumatics
	public static final int ELEVATOR_CLIMB_SHIFT_PCM_ID = 3;
	public static final int FRONT_LEG_SHIFT_PCM_ID = 4;
	public static final int BACK_LEG_SHIFT_PCM_ID = 5;

	// DIO
	public static final int INTAKE_FRONT_IR_SENSOR_DIO_ID = 0;
	public static final int INTAKE_FRONT_VEX_SENSOR_DIO_ID = 3;

}
