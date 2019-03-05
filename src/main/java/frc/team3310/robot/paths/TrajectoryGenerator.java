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

        // 2 Hatch Auto (Front/Back)
        public static final Pose2d kRocketTrackPose = new Pose2d(new Translation2d(183, -126),
                        Rotation2d.fromDegrees(-30.00));

        public static final Pose2d kRocketFrontPose = new Pose2d(new Translation2d(201, -135),
                        Rotation2d.fromDegrees(-30.00));

        public static final Pose2d kRocketFrontToTurn1Pose = new Pose2d(new Translation2d(166, -100),
                        Rotation2d.fromDegrees(-100.00));

        public static final Pose2d kTurn1ToTurn2Pose = new Pose2d(new Translation2d(215, -100),
                        Rotation2d.fromDegrees(-160.00));

        public static final Pose2d kTurn1ToLoadingPose = new Pose2d(new Translation2d(30, -135),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kLoadingPose = new Pose2d(new Translation2d(20.00, -135),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kLoadingGrabPose = new Pose2d(new Translation2d(0.00, -135),
                        Rotation2d.fromDegrees(-180.0));

        public static final Pose2d kRocketBackPose = new Pose2d(new Translation2d(300, -110),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kRocketBackToTurnPose = new Pose2d(new Translation2d(250, -145),
                        Rotation2d.fromDegrees(-140.00));

        public static final Pose2d kCargoTrackPoseSideStart = new Pose2d(new Translation2d(195, -15),
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kCargoToTurn1Pose = new Pose2d(new Translation2d(175, -90),
                        Rotation2d.fromDegrees(-60.00));

        public static final Pose2d kTurn1CargoToLoadingPose = new Pose2d(new Translation2d(30, -135),
                        Rotation2d.fromDegrees(-180.00));

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
                public final MirroredTrajectory loadingToRocketBack;
                public final MirroredTrajectory rocketFrontToTurn1;
                public final MirroredTrajectory turn1ToTurn2;
                public final MirroredTrajectory turn2ToLoading;
                public final MirroredTrajectory turn3ToRocketBack;
                public final MirroredTrajectory level1StartToCargoFront;
                public final MirroredTrajectory cargoToTurn1;
                public final MirroredTrajectory turn1ToLoading;

                private TrajectorySet() {
                        level1StartToRocketFront = new MirroredTrajectory(getLevel1StartToRocketFront());
                        loadingToRocketBack = new MirroredTrajectory(getLoadingToRocketBack());
                        rocketFrontToTurn1 = new MirroredTrajectory(getRocketFrontToTurn1());
                        turn1ToTurn2 = new MirroredTrajectory(getTurn1ToTurn2());
                        turn2ToLoading = new MirroredTrajectory(getTurn2ToLoading());
                        turn3ToRocketBack = new MirroredTrajectory(getTurn3ToRocketBack());
                        level1StartToCargoFront = new MirroredTrajectory(getLevel1StartToCargoFront());
                        cargoToTurn1 = new MirroredTrajectory(getCargoFrontToWall());
                        turn1ToLoading = new MirroredTrajectory(getTurn1ToLoading());

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
                                        kSimpleSwitchMaxVelocity * .3, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1StartToRocketFront() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel1);
                        waypoints.add(new Pose2d(100.7, -48.4, Rotation2d.fromDegrees(0.0)));
                        waypoints.add(kRocketTrackPose
                                        .transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 4.0))));

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getRocketFrontToTurn1() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kRocketFrontPose);
                        waypoints.add(kRocketFrontToTurn1Pose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getTurn1ToTurn2() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kRocketFrontToTurn1Pose);
                        waypoints.add(kTurn1ToTurn2Pose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getTurn2ToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kTurn1ToTurn2Pose);
                        waypoints.add(kTurn1ToLoadingPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingToRocketBack() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kLoadingGrabPose);
                        waypoints.add(kRocketBackPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getTurn3ToRocketBack() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kRocketBackPose);
                        waypoints.add(kRocketBackToTurnPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1StartToCargoFront() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel1);
                        waypoints.add(new Pose2d(100.7, -48.4, Rotation2d.fromDegrees(0.0)));
                        waypoints.add(kCargoTrackPoseSideStart
                                        .transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 4.0))));

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFrontToWall() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoTrackPoseSideStart);
                        waypoints.add(kCargoToTurn1Pose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getTurn1ToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoToTurn1Pose);
                        waypoints.add(kLoadingPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }
        }
}
