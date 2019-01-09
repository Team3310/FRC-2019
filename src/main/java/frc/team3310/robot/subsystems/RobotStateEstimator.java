package frc.team3310.robot.subsystems;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team3310.robot.Kinematics;
import frc.team3310.robot.loops.ILooper;
import frc.team3310.robot.loops.Loop;
import frc.team3310.utility.lib.control.RobotStatus;
import frc.team3310.utility.lib.geometry.Rotation2d;
import frc.team3310.utility.lib.geometry.Twist2d;

public class RobotStateEstimator extends Subsystem {
    public static RobotStateEstimator instance_ = new RobotStateEstimator();
    private RobotStatus robot_state_ = RobotStatus.getInstance();
    private Drive drive_ = Drive.getInstance();
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;
    private double back_encoder_prev_distance_ = 0.0;

    RobotStateEstimator() {
    }

    @Override
    public void initDefaultCommand() {
        
    }

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    public boolean checkSystem() {
        return false;
    }

    public void outputTelemetry() {
        // No-op
    }

    public void stop() {
        // No-op
    }

    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    private class EnabledLoop implements Loop {
        @Override
        public synchronized void onStart(double timestamp) {
            left_encoder_prev_distance_ = drive_.getLeftPositionInches();
            right_encoder_prev_distance_ = drive_.getRightPositionInches();

        }

        @Override
        public synchronized void onLoop(double timestamp) {
            final double left_distance = drive_.getLeftPositionInches();
            final double right_distance = drive_.getRightPositionInches();
            final double delta_left = left_distance - left_encoder_prev_distance_;
            final double delta_right = right_distance - right_encoder_prev_distance_;
            final Rotation2d gyro_angle = drive_.getHeading();
            final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                    delta_left, delta_right, gyro_angle);
            final Twist2d predicted_velocity = Kinematics.forwardKinematics(drive_.getLeftVelocityInchesPerSec(),
                    drive_.getRightVelocityInchesPerSec());
            robot_state_.addObservations(timestamp, odometry_velocity,
                    predicted_velocity);
            left_encoder_prev_distance_ = left_distance;
            right_encoder_prev_distance_ = right_distance;
        }

        @Override
        public void onStop(double timestamp) {
            // no-op
        }

    }
}

