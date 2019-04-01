package com.team3310.frc2019.planners;

import java.util.Arrays;

import org.junit.jupiter.api.Test;

import frc.team3310.robot.Constants;
import frc.team3310.robot.Kinematics;
import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.robot.planners.DriveMotionPlanner;
import frc.team3310.utility.lib.geometry.Pose2d;
import frc.team3310.utility.lib.geometry.Rotation2d;
import frc.team3310.utility.lib.geometry.Translation2d;
import frc.team3310.utility.lib.geometry.Twist2d;
import frc.team3310.utility.lib.trajectory.LazyLoadTrajectory;
import frc.team3310.utility.lib.trajectory.TimedView;
import frc.team3310.utility.lib.trajectory.TrajectoryIterator;
import frc.team3310.utility.lib.trajectory.timing.CentripetalAccelerationConstraint;

public class DriveMotionPlannerTest {

    private Pose2d kCargoSideTurnToLoadingPose = new Pose2d(new Translation2d(295.75, -60.5), Rotation2d.fromDegrees(-180.00));
    private Pose2d kLoadingPose = new Pose2d(new Translation2d(30.00, -130), Rotation2d.fromDegrees(-180.00));


    @Test
    public void testStraight() {
        TrajectoryGenerator.getInstance().generateTrajectories();

        DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.PURE_PURSUIT);
        LazyLoadTrajectory lazyTrajectory = TrajectoryGenerator.getInstance().getTrajectorySet().driveStraight;
        lazyTrajectory.activate();
        motion_planner.setTrajectory(new TrajectoryIterator<>(
                new TimedView<>(lazyTrajectory.getTrajectory().right)));

        double t = 0.0;
        Pose2d pose = motion_planner.setpoint().state().getPose();
        System.out.println("t," + motion_planner.toCSVHeader());
        while (!motion_planner.isDone()) {
            motion_planner.update(t, pose);
            pose = motion_planner.mSetpoint.state().getPose();// .transformBy(new Pose2d(new Translation2d(0.0, 1.0),
            // Rotation2d.fromDegrees(2.0)));

            System.out.println(t + "," + motion_planner.toCSV());
            t += 0.01;// + (2.0 * Math.random() - 1.0) * 0.002;
        }
    }

    @Test
    public void testForwardSwerveRight() {
        DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.PURE_PURSUIT);
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory(false,
                Arrays.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(120.0, -36.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(240.0, -36.0), Rotation2d.identity())),
                Arrays.asList(new CentripetalAccelerationConstraint(120.0)), 120.0, 120.0, 10.0))));

        double t = 0.0;
        Pose2d pose = motion_planner.setpoint().state().getPose();
        System.out.println("t," + motion_planner.toCSVHeader());
        while (!motion_planner.isDone()) {
            motion_planner.update(t, pose);
            pose = motion_planner.mSetpoint.state().getPose();// .transformBy(new Pose2d(new Translation2d(0.0, 1.0),
            // Rotation2d.fromDegrees(2.0)));

            System.out.println(t + "," + motion_planner.toCSV());
            t += 0.01;// + (2.0 * Math.random() - 1.0) * 0.002;
        }
    }

    @Test
    public void testForwardSwerveLeft() {
        DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.FEEDFORWARD_ONLY);
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory(false,
                Arrays.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(120.0, 36.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(240.0, 0.0), Rotation2d.identity())),
                null, 120.0, 120.0, 10.0))));
        double t = 0.0;
        Pose2d pose = motion_planner.setpoint().state().getPose();
        while (!motion_planner.isDone()) {
            motion_planner.update(t, pose);
            pose = motion_planner.mSetpoint.state().getPose();// .transformBy(new Pose2d(new Translation2d(0.0, -1.0),
            // Rotation2d.fromDegrees(-2.0)));
            System.out.println(t + "," + motion_planner.toCSV());
            t += 0.01;
        }
    }

    @Test
    public void testForwardWithTheta180() {

        DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.NONLINEAR_FEEDBACK);
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory
                (false, Arrays.asList(kCargoSideTurnToLoadingPose, kLoadingPose),
                        null,
                        120.0, 120.0, 10.0))));
        double t = 0.0;
        Pose2d pose = motion_planner.setpoint().state().getPose();
        System.out.println(motion_planner.toCSVHeader());
        while (!motion_planner.isDone()) {
            motion_planner.update(t, pose);
            pose = motion_planner.mSetpoint.state().getPose();//.transformBy(new Pose2d(new Translation2d(0.0, -1.0),
            // Rotation2d.fromDegrees(-2.0)));
            System.out.println(motion_planner.toCSV());
            t += 0.01;
        }
    }

    @Test
    public void testReverseSwerveLeft() {
        DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory(true,
                Arrays.asList(new Pose2d(new Translation2d(240.0, 0.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(120.0, 36.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.identity())),
                null, 120.0, 120.0, 10.0))));
        double t = 0.0;
        Pose2d pose = motion_planner.setpoint().state().getPose();
        while (!motion_planner.isDone()) {
            motion_planner.update(t, pose);
            pose = motion_planner.mSetpoint.state().getPose();
            System.out.println(t + "," + motion_planner.toCSV());
            t += 0.01;
        }
    }

    @Test
    public void testForwardReverseSame() {
        DriveMotionPlanner fwd_motion_planner = new DriveMotionPlanner();
        fwd_motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.PURE_PURSUIT);
        fwd_motion_planner
                .setTrajectory(new TrajectoryIterator<>(new TimedView<>(fwd_motion_planner.generateTrajectory(false,
                        Arrays.asList(new Pose2d(new Translation2d(10.0, 10.0), Rotation2d.fromDegrees(5.0)),
                                new Pose2d(new Translation2d(120.0, 36.0), Rotation2d.identity()),
                                new Pose2d(new Translation2d(230.0, 10.0), Rotation2d.fromDegrees(-5.0))),
                        null, 120.0, 120.0, 5.0))));
        DriveMotionPlanner rev_motion_planner = new DriveMotionPlanner();
        rev_motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.PURE_PURSUIT);
        rev_motion_planner
                .setTrajectory(new TrajectoryIterator<>(new TimedView<>(rev_motion_planner.generateTrajectory(true,
                        Arrays.asList(new Pose2d(new Translation2d(230.0, 10.0), Rotation2d.fromDegrees(-5.0)),
                                new Pose2d(new Translation2d(120.0, 36.0), Rotation2d.identity()),
                                new Pose2d(new Translation2d(10.0, 10.0), Rotation2d.fromDegrees(5.0))),
                        null, 120.0, 120.0, 5.0))));

        final double dt = 0.01;
        double t = 0.0;
        Pose2d start_error_fwd = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(2.0));
        Pose2d start_error_rev = new Pose2d(-2.0, 3.0, Rotation2d.fromDegrees(-2.0));
        Pose2d fwd_pose = fwd_motion_planner.setpoint().state().getPose().transformBy(start_error_fwd);
        Pose2d rev_pose = rev_motion_planner.setpoint().state().getPose().transformBy(start_error_rev);
        while (!fwd_motion_planner.isDone() || !rev_motion_planner.isDone()) {
            DriveMotionPlanner.Output fwd_output = fwd_motion_planner.update(t, fwd_pose);
            Twist2d fwd_delta = Kinematics.forwardKinematics(
                    fwd_output.left_velocity * dt * Constants.kDriveWheelDiameterInches / 2.0,
                    fwd_output.right_velocity * dt * Constants.kDriveWheelDiameterInches / 2.0);
            fwd_pose = fwd_pose.transformBy(Pose2d.exp(fwd_delta));
            // System.out.println("FWD Delta: " + fwd_delta + ", Pose: " + fwd_pose);
            DriveMotionPlanner.Output rev_output = rev_motion_planner.update(t, rev_pose);
            Twist2d rev_delta = Kinematics.forwardKinematics(
                    rev_output.left_velocity * dt * Constants.kDriveWheelDiameterInches / 2.0,
                    rev_output.right_velocity * dt * Constants.kDriveWheelDiameterInches / 2.0);
            rev_pose = rev_pose.transformBy(Pose2d.exp(rev_delta));
            // System.out.println("REV Delta: " + rev_delta + ", Pose: " + rev_pose);
            System.out.println(fwd_motion_planner.toCSV() + "," + rev_motion_planner.toCSV());
            t += dt;
        }
    }

    @Test
    public void testForwardReverseSame180Theta() {
        DriveMotionPlanner fwd_motion_planner = new DriveMotionPlanner();
        fwd_motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.NONLINEAR_FEEDBACK);
        fwd_motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(fwd_motion_planner.generateTrajectory
                (false, Arrays.asList(kCargoSideTurnToLoadingPose, kLoadingPose),
                        null,
                        120.0, 120.0, 5.0))));
        DriveMotionPlanner rev_motion_planner = new DriveMotionPlanner();
        rev_motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.NONLINEAR_FEEDBACK);
        rev_motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(rev_motion_planner.generateTrajectory
                (true, Arrays.asList(kLoadingPose, kCargoSideTurnToLoadingPose),
                        null,
                        120.0, 120.0, 5.0))));

        final double dt = 0.01;
        double t = 0.0;
        Pose2d start_error_fwd = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(2.0));
        Pose2d start_error_rev = new Pose2d(-2.0, 3.0, Rotation2d.fromDegrees(-2.0));
        Pose2d fwd_pose = fwd_motion_planner.setpoint().state().getPose().transformBy(start_error_fwd);
        Pose2d rev_pose = rev_motion_planner.setpoint().state().getPose().transformBy(start_error_rev);
        while (!fwd_motion_planner.isDone() || !rev_motion_planner.isDone()) {
            DriveMotionPlanner.Output fwd_output = fwd_motion_planner.update(t, fwd_pose);
            Twist2d fwd_delta = Kinematics.forwardKinematics(fwd_output.left_velocity * dt * Constants
                    .kDriveWheelDiameterInches / 2.0, fwd_output.right_velocity * dt * Constants
                    .kDriveWheelDiameterInches / 2.0);
            fwd_pose = fwd_pose.transformBy(Pose2d.exp(fwd_delta));
            //System.out.println("FWD Delta: " + fwd_delta + ", Pose: " + fwd_pose);
            DriveMotionPlanner.Output rev_output = rev_motion_planner.update(t, rev_pose);
            Twist2d rev_delta = Kinematics.forwardKinematics(rev_output.left_velocity * dt * Constants
                    .kDriveWheelDiameterInches / 2.0, rev_output.right_velocity * dt * Constants
                    .kDriveWheelDiameterInches / 2.0);
            rev_pose = rev_pose.transformBy(Pose2d.exp(rev_delta));
            //System.out.println("REV Delta: " + rev_delta + ", Pose: " + rev_pose);
            System.out.println(fwd_motion_planner.toCSV() + "," + rev_motion_planner.toCSV());
            t += dt;
        }
    }

    @Test
    public void testFollowerReachesGoalTheta180() {
        final DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.PURE_PURSUIT);
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory(false,
                Arrays.asList(kCargoSideTurnToLoadingPose, kLoadingPose), null, 120.0, 120.0, 10.0))));
        final double dt = 0.01;
        double t = 0.0;
        Pose2d initial_error = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(3.5));
        Pose2d pose = motion_planner.setpoint().state().getPose().transformBy(initial_error);
        System.out.println(motion_planner.setpoint().toCSVHeader() + ", leftOutputVel, rightOutputVel," + initial_error.toCSVHeader());
        while (!motion_planner.isDone()) {
            DriveMotionPlanner.Output output = motion_planner.update(t, pose);
            Twist2d delta = Kinematics.forwardKinematics(output.left_velocity * dt * Constants
                    .kDriveWheelDiameterInches / 2.0, output.right_velocity * dt * Constants
                    .kDriveWheelDiameterInches / 2.0);
            // Add some systemic error.
            delta = new Twist2d(delta.dx * 1.0, delta.dy * 1.0, delta.dtheta * 1.05);
            pose = pose.transformBy(Pose2d.exp(delta));
            t += dt;
            System.out.println(motion_planner.setpoint().toCSV() + "," + output.left_velocity + "," + output.right_velocity + "," + motion_planner.error().toCSV());
        }
        System.out.println(pose);
    }

    @Test
    public void testFollowerReachesGoal() {
        final DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.NONLINEAR_FEEDBACK);
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory(false,
                Arrays.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(120.0, -36.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(240.0, -36.0), Rotation2d.identity())),
                null, 120.0, 120.0, 10.0))));
        final double dt = 0.01;
        double t = 0.0;
        Pose2d initial_error = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(3.5));
        Pose2d pose = motion_planner.setpoint().state().getPose().transformBy(initial_error);
        while (!motion_planner.isDone()) {
            DriveMotionPlanner.Output output = motion_planner.update(t, pose);
            Twist2d delta = Kinematics.forwardKinematics(
                    output.left_velocity * dt * Constants.kDriveWheelDiameterInches / 2.0,
                    output.right_velocity * dt * Constants.kDriveWheelDiameterInches / 2.0);
            // Add some systemic error.
            delta = new Twist2d(delta.dx * 1.0, delta.dy * 1.0, delta.dtheta * 1.05);
            pose = pose.transformBy(Pose2d.exp(delta));
            t += dt;
            System.out.println(motion_planner.setpoint().toCSV() + "," + pose.toCSV());
        }
        System.out.println(pose);
    }

    @Test
    public void testVoltages() {
        final DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory(false,
                Arrays.asList(Pose2d.identity(), Pose2d.fromTranslation(new Translation2d(48.0, 0.0)),
                        new Pose2d(new Translation2d(96.0, 48.0), Rotation2d.fromDegrees(90.0)),
                        new Pose2d(new Translation2d(96.0, 96.0), Rotation2d.fromDegrees(90.0))),
                null, 48.0, 48.0, 10.0))));
        double t = 0.0;
        Pose2d pose = motion_planner.setpoint().state().getPose()
                .transformBy(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180.0)));
        while (!motion_planner.isDone()) {
            motion_planner.update(t, pose);
            pose = motion_planner.mSetpoint.state().getPose();
            System.out.println(t + "," + motion_planner.toCSV());
            t += 0.01;
        }
    }
}
