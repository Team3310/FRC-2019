package frc.team3310.robot.paths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.team3310.robot.planners.DriveMotionPlanner;
import frc.team3310.utility.lib.geometry.Pose2d;
import frc.team3310.utility.lib.geometry.Pose2dWithCurvature;
import frc.team3310.utility.lib.geometry.Rotation2d;
import frc.team3310.utility.lib.geometry.Translation2d;
import frc.team3310.utility.lib.trajectory.Trajectory;
import frc.team3310.utility.lib.trajectory.TrajectoryUtil;
import frc.team3310.utility.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.team3310.utility.lib.trajectory.timing.TimedState;
import frc.team3310.utility.lib.trajectory.timing.TimingConstraint;

//TODO CHANGE ALL VALUES FOR 2019 FIELD/ROBOT
public class TrajectoryGenerator {
        private static final double kMaxVelocity = 130.0;
        private static final double kMaxAccel = 130.0;
        private static final double kMaxCentripetalAccelElevatorDown = 110.0;
        private static final double kMaxCentripetalAccel = 100.0;
        private static final double kMaxVoltage = 9.0;
        private static final double kFirstPathMaxVoltage = 9.0;
        private static final double kFirstPathMaxAccel = 130.0;
        private static final double kFirstPathMaxVel = 130.0;

        private static final double kSimpleSwitchMaxAccel = 100.0;
        private static final double kSimpleSwitchMaxCentripetalAccel = 80.0;
        private static final double kSimpleSwitchMaxVelocity = 120.0;

        private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
        private final DriveMotionPlanner mMotionPlanner;
        private TrajectorySet mTrajectorySet = null;

        public static TrajectoryGenerator getInstance() {
                return mInstance;
        }

        private TrajectoryGenerator() {
                mMotionPlanner = new DriveMotionPlanner();
        }

        public void generateTrajectories() {
                if (mTrajectorySet == null) {
                        System.out.println("Generating trajectories...");
                        mTrajectorySet = new TrajectorySet();
                        System.out.println("Finished trajectory generation");
                }
        }

        public TrajectorySet getTrajectorySet() {
                return mTrajectorySet;
        }

        public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed,
                        final List<Pose2d> waypoints, final List<TimingConstraint<Pose2dWithCurvature>> constraints,
                        double max_vel, // inches/s
                        double max_accel, // inches/s^2
                        double max_voltage) {
                return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel,
                                max_voltage);
        }

        public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed,
                        final List<Pose2d> waypoints, final List<TimingConstraint<Pose2dWithCurvature>> constraints,
                        double start_vel, // inches/s
                        double end_vel, // inches/s
                        double max_vel, // inches/s
                        double max_accel, // inches/s^2
                        double max_voltage) {
                return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel,
                                max_accel, max_voltage);
        }

        // CRITICAL POSES
        // Origin is the center of the robot when the robot is placed against the middle
        // of the alliance station wall.
        // +x is towards the center of the field.
        // +y is to the left.
        // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x
        // axis for LEFT)
        public static final Pose2d kSideStartLevel1 = new Pose2d(63.7, -48.4, Rotation2d.fromDegrees(0.0));

        public static final Pose2d kCenterToIntake = new Pose2d(new Translation2d(-24.0, 0.0), Rotation2d.identity());

        public static final Pose2d kSideStartLevel1Intake = kSideStartLevel1.transformBy(kCenterToIntake);

        // STARTING IN CENTER
        public static final Pose2d kCenterStartPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
        public static final Pose2d kSimpleSwitchStartPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
        public static final Pose2d kRightSwitchPose = new Pose2d(new Translation2d(100.0, 0.0),
                        Rotation2d.fromDegrees(0.0));
        public static final Pose2d kLeftSwitchPose = new Pose2d(new Translation2d(100.0, 0.0),
                        Rotation2d.fromDegrees(0.0));

        public static final Pose2d kRocketFrontPose = new Pose2d(new Translation2d(201, -135),
                        Rotation2d.fromDegrees(-30.00));
        public static final Pose2d kRocketBackPose = new Pose2d(new Translation2d(256, -135),
                        Rotation2d.fromDegrees(150.00));
        public static final Pose2d kRocketTrackPose = new Pose2d(new Translation2d(183, -126),
                        Rotation2d.fromDegrees(-30.00));

        public static final Pose2d kLoadingPose = new Pose2d(new Translation2d(20.00, -135),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kLoadingReversePose = new Pose2d(new Translation2d(30.00, -135),
                        Rotation2d.fromDegrees(0.0));

        public class TrajectorySet {
                public class MirroredTrajectory {
                        public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                                this.right = right;
                                this.left = TrajectoryUtil.mirrorTimed(right);
                        }

                        public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                                return left ? this.left : this.right;
                        }

                        public final Trajectory<TimedState<Pose2dWithCurvature>> left;
                        public final Trajectory<TimedState<Pose2dWithCurvature>> right;
                }

                public final Trajectory<TimedState<Pose2dWithCurvature>> centerStartToLeftSwitch;
                public final Trajectory<TimedState<Pose2dWithCurvature>> centerStartToRightSwitch;
                public final Trajectory<TimedState<Pose2dWithCurvature>> simpleStartToLeftSwitch;
                public final Trajectory<TimedState<Pose2dWithCurvature>> simpleStartToRightSwitch;
                public final MirroredTrajectory level1StartToRocketFront;
                public final MirroredTrajectory rocketFrontToLoading;
                public final MirroredTrajectory loadingToRocketBack;

                private TrajectorySet() {
                        level1StartToRocketFront = new MirroredTrajectory(getLevel1StartToRocketFront());
                        rocketFrontToLoading = new MirroredTrajectory(getRocketFrontToLoading());
                        loadingToRocketBack = new MirroredTrajectory(getLoadingToRocketBack());

                        centerStartToLeftSwitch = getCenterStartToLeftSwitch();
                        centerStartToRightSwitch = getCenterStartToRightSwitch();
                        simpleStartToLeftSwitch = getSimpleStartToLeftSwitch();
                        simpleStartToRightSwitch = getSimpleStartToRightSwitch();

                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCenterStartToLeftSwitch() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCenterStartPose);
                        waypoints.add(kLeftSwitchPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                                        kMaxVelocity, kMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCenterStartToRightSwitch() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCenterStartPose);
                        waypoints.add(kRightSwitchPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                                        kMaxVelocity, kMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getSimpleStartToRightSwitch() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSimpleSwitchStartPose);
                        waypoints.add(kRightSwitchPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getSimpleStartToLeftSwitch() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSimpleSwitchStartPose);
                        waypoints.add(kLeftSwitchPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity *  .3, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1StartToRocketFront() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel1);
                        waypoints.add(new Pose2d(100.7, -48.4, Rotation2d.fromDegrees(0.0)));
                        waypoints.add(kRocketFrontPose
                                        .transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 4.0))));

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getRocketFrontToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kRocketFrontPose);
                        waypoints.add(kLoadingReversePose
                                        .transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 4.0))));

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingToRocketBack() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kLoadingPose);
                        waypoints.add(kRocketBackPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }
        }
}
