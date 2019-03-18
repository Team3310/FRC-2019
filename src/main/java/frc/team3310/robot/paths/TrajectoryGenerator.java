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
        public static enum RightLeftAutonSide {
                RIGHT, LEFT
        };

        private static final double kMaxVelocity = 130.0;
        private static final double kMaxAccel = 130.0;
        private static final double kMaxCentripetalAccelElevatorDown = 110.0;
        private static final double kMaxCentripetalAccel = 100.0;
        private static final double kMaxVoltage = 9.0;
        private static final double kFirstPathMaxVoltage = 9.0;
        private static final double kFirstPathMaxAccel = 130.0;
        private static final double kFirstPathMaxVel = 130.0;

        private static final double kSimpleSwitchMaxAccel = 100.0;
        private static final double kSimpleSwitchMaxCentripetalAccel = 110.0;
        private static final double kSimpleSwitchMaxVelocity = 144.0;

        private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
        private final DriveMotionPlanner mMotionPlanner;
        private TrajectorySet mTrajectorySet = null;
        private RightLeftAutonSide rightLeftSide = RightLeftAutonSide.RIGHT;

        public static TrajectoryGenerator getInstance() {
                return mInstance;
        }

        private TrajectoryGenerator() {
                mMotionPlanner = new DriveMotionPlanner();
        }

        public void setRightLeftAutonSide(RightLeftAutonSide side) {
                this.rightLeftSide = side;
        }

        public RightLeftAutonSide getRightLeftAutonSide() {
                return rightLeftSide;
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

        // OLD
        public static final Pose2d kCenterStartPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
        public static final Pose2d kSimpleSwitchStartPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
        public static final Pose2d kRightSwitchPose = new Pose2d(new Translation2d(100.0, 0.0),
                        Rotation2d.fromDegrees(0.0));
        public static final Pose2d kLeftSwitchPose = new Pose2d(new Translation2d(10.0, 0.0),
                        Rotation2d.fromDegrees(0.0));

        // CRITICAL POSES
        // Origin is the center of the robot when the robot is placed against the middle
        // of the alliance station wall.
        // +x is towards the center of the field.
        // +y is to the left.
        // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x
        // axis for LEFT)
        public static final Pose2d kSideStartLevel1 = new Pose2d(63.7, -48.4, Rotation2d.fromDegrees(0.0));

        public static final Pose2d kSideStartLevel2 = new Pose2d(15.75, -48.4, Rotation2d.fromDegrees(0.0));

        public static final Pose2d kSideStartHangingLevel2 = new Pose2d(53.25, -48.4, Rotation2d.fromDegrees(0.0));

        public static final Pose2d kCenterToIntake = new Pose2d(new Translation2d(-18.0, 0.0), Rotation2d.identity());

        public static final Pose2d kSideStartLevel1Intake = kSideStartLevel1.transformBy(kCenterToIntake);

        // 2 Hatch Auto (Front/Back)
        public static final Pose2d kRocketTrackPose = new Pose2d(new Translation2d(183, -132),
                        Rotation2d.fromDegrees(-30.00));

        public static final Pose2d kRocketFrontPose = new Pose2d(new Translation2d(201, -135),
                        Rotation2d.fromDegrees(-30.00));

        public static final Pose2d kLoadingTrackPose = new Pose2d(new Translation2d(20.00, -135),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kLoadingGrabPose = new Pose2d(new Translation2d(0.00, -135),
                        Rotation2d.fromDegrees(-180.0));

        public static final Pose2d kLoadingToMidRocketBack = new Pose2d(new Translation2d(230, -110),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kRocketBackPoseToTurnPose = new Pose2d(new Translation2d(275, -110),
                        Rotation2d.fromDegrees(-147.00));

        public static final Pose2d kRocketBackPose = new Pose2d(new Translation2d(270, -140),
                        Rotation2d.fromDegrees(-150.00));

        // Cargo 2 Hatch (Middle/Side)

        public static final Pose2d kCargoFrontTrackPoseSideStart = new Pose2d(new Translation2d(195, -20),
                        Rotation2d.fromDegrees(0.00));
        public static final Pose2d kCargoFrontPose = new Pose2d(new Translation2d(207.7, -10),
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kCargoFrontToLoadingMidPose = new Pose2d(new Translation2d(115, -135),
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kCargoFrontToLoadingTrackPose = new Pose2d(new Translation2d(30, -135),
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kCargoSide1Pose = new Pose2d(new Translation2d(315.75, -60.5), // Near 275 //Mid 295
                                                                                                  // Far 315
                        Rotation2d.fromDegrees(90.00));

        public static final Pose2d kLoadingToCargoFront2Pose = new Pose2d(new Translation2d(185, 20),
                        Rotation2d.fromDegrees(-90.00));

        public static final Pose2d kCargoTurn1LoadingToTrackPose = new Pose2d(new Translation2d(200, 14),
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kTurn1CargoSide1Pose = new Pose2d(new Translation2d(295, -115),
                        Rotation2d.fromDegrees(135.00));

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

                public final MirroredTrajectory level1StartToRocketFront;
                public final MirroredTrajectory level1StartToCargoFront;
                public final MirroredTrajectory level2StartToRocketFront;
                public final MirroredTrajectory level2StartToCargoFront;
                public final MirroredTrajectory cargoFrontToLoadingReversed;
                public final MirroredTrajectory loadingToRocketBack;
                public final MirroredTrajectory loadingTocargoFrontTrack2;
                public final MirroredTrajectory loadingToTurn1CargoSide1;
                public final MirroredTrajectory turn1ToCargoSide1;
                public final MirroredTrajectory turn3ToRocketBack;
                public final MirroredTrajectory track2PoseToCargo2;
                public final MirroredTrajectory centerStartToLeftSwitch;
                public final MirroredTrajectory centerStartToRightSwitch;
                public final MirroredTrajectory simpleStartToLeftSwitch;
                public final MirroredTrajectory simpleStartToRightSwitch;

                private TrajectorySet() {
                        level1StartToRocketFront = new MirroredTrajectory(getLevel1StartToRocketFront());
                        level1StartToCargoFront = new MirroredTrajectory(getLevel1StartToCargoFront());
                        level2StartToRocketFront = new MirroredTrajectory(getLevel2StartToRocketFront());
                        level2StartToCargoFront = new MirroredTrajectory(getLevel2StartToCargoFront());
                        cargoFrontToLoadingReversed = new MirroredTrajectory(getCargoFrontToLoadingReversed());
                        loadingToRocketBack = new MirroredTrajectory(getLoadingToRocketBack());
                        loadingTocargoFrontTrack2 = new MirroredTrajectory(getLoadingToCargoFrontTrack2());
                        loadingToTurn1CargoSide1 = new MirroredTrajectory(getLoadingToTurn1CargoSide1());
                        turn1ToCargoSide1 = new MirroredTrajectory(getTurn1ToCargoSide1());
                        turn3ToRocketBack = new MirroredTrajectory(getTurn3ToRocketBack());
                        track2PoseToCargo2 = new MirroredTrajectory(getCargoTrack2ToCargocScorePose());
                        centerStartToLeftSwitch = new MirroredTrajectory(getCenterStartToLeftSwitch());
                        centerStartToRightSwitch = new MirroredTrajectory(getCenterStartToRightSwitch());
                        simpleStartToLeftSwitch = new MirroredTrajectory(getSimpleStartToLeftSwitch());
                        simpleStartToRightSwitch = new MirroredTrajectory(getSimpleStartToRightSwitch());
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

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1StartToCargoFront() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel1);
                        waypoints.add(new Pose2d(100.7, -48.4, Rotation2d.fromDegrees(0.0)));
                        waypoints.add(kCargoFrontTrackPoseSideStart
                                        .transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 4.0))));

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFrontToLoadingReversed() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoFrontPose);
                        waypoints.add(kCargoFrontToLoadingMidPose);
                        waypoints.add(kCargoFrontToLoadingTrackPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingToRocketBack() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kLoadingGrabPose);
                        waypoints.add(kLoadingToMidRocketBack);
                        waypoints.add(kRocketBackPoseToTurnPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getTurn3ToRocketBack() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kRocketBackPoseToTurnPose);
                        waypoints.add(kRocketBackPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingToTurn1CargoSide1() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kLoadingTrackPose);
                        waypoints.add(kTurn1CargoSide1Pose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getTurn1ToCargoSide1() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kTurn1CargoSide1Pose);
                        waypoints.add(kCargoSide1Pose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingToCargoFrontTrack2() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kLoadingGrabPose);
                        waypoints.add(kLoadingToCargoFront2Pose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTrack2ToCargocScorePose() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kLoadingToCargoFront2Pose);
                        waypoints.add(kCargoTurn1LoadingToTrackPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                // Lvl 2
                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2StartToRocketFront() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel2);
                        waypoints.add(new Pose2d(100.7, -48.4, Rotation2d.fromDegrees(0.0)));
                        waypoints.add(kRocketTrackPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2StartToCargoFront() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartHangingLevel2);
                        waypoints.add(new Pose2d(100.7, -48.4, Rotation2d.fromDegrees(0.0)));
                        waypoints.add(kCargoFrontTrackPoseSideStart
                                        .transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 4.0))));

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }
        }
}
