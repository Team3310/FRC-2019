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
        private static final double kMaxVoltage = 10.0;
        private static final double kFirstPathMaxVoltage = 9.0;
        private static final double kFirstPathMaxAccel = 130.0;
        private static final double kFirstPathMaxVel = 130.0;

        private static final double kSimpleSwitchMaxAccel = 140.0;
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

        // CRITICAL POSES
        // Origin is the center of the robot when the robot is placed against the middle
        // of the alliance station wall.
        // +x is towards the center of the field.
        // +y is to the left.
        // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x
        // axis for LEFT)
        public static final Pose2d kSideStartLevel1 = new Pose2d(63.7, -48.4, Rotation2d.fromDegrees(0.0));

        public static final Pose2d kSideStartLevel1Reversed = new Pose2d(63.7, -48.4, Rotation2d.fromDegrees(-180.0));

        public static final Pose2d kSideStartLevel2 = new Pose2d(32.118, -48.4, Rotation2d.fromDegrees(0.0));

        public static final Pose2d kCenterToIntake = new Pose2d(new Translation2d(-18.0, 0.0), Rotation2d.identity());

        public static final Pose2d kSideStartLevel1Intake = kSideStartLevel1.transformBy(kCenterToIntake);

        public static final Pose2d kDriveStraightFrontPlatfrom = new Pose2d(new Translation2d(100, -48.4),
                        Rotation2d.fromDegrees(0.0));

        // 2 Hatch Auto (Front/Back)
        public static final Pose2d kRocketFrontTrackPose = new Pose2d(new Translation2d(183, -127),
                        Rotation2d.fromDegrees(-30.00));
        public static final Pose2d kRocketFrontScorePose = new Pose2d(new Translation2d(201, -135),
                        Rotation2d.fromDegrees(-30.00));

        public static final Pose2d kRocketFrontTurnPose = new Pose2d(new Translation2d(180, -80),
                        Rotation2d.fromDegrees(-105.00));

        public static final Pose2d kRocketBackTurnPose = new Pose2d(new Translation2d(310, -130),
                        Rotation2d.fromDegrees(160.00));

        public static final Pose2d kRocketBackScorePose = new Pose2d(new Translation2d(258, -138),
                        Rotation2d.fromDegrees(-150.00));

        public static final Pose2d kLoadingTrackPose = new Pose2d(new Translation2d(30.00, -135),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kLoadingGrabPose = new Pose2d(new Translation2d(0.00, -135),
                        Rotation2d.fromDegrees(-180.0));

        public static final Pose2d kLoadingReversedTrackPose = new Pose2d(new Translation2d(40.00, -135),
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kLoadingReversedGrabPose = new Pose2d(new Translation2d(0.00, -135),
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kMidRocketBackPose = new Pose2d(new Translation2d(230, -110),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kCargoFrontPose = new Pose2d(new Translation2d(215, -14), // 207.7
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kCargoFrontTurn1Pose = new Pose2d(new Translation2d(175, -90),
                        Rotation2d.fromDegrees(90.00));

        public static final Pose2d kCargo2MidPose = new Pose2d(new Translation2d(185, 0), // 200
                        Rotation2d.fromDegrees(-90.00));

        public static final Pose2d kCargoFront2TrackPose = new Pose2d(new Translation2d(155, 10), // 170
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kCargoFrontScore2Pose = new Pose2d(new Translation2d(230, 10), // 207
                        Rotation2d.fromDegrees(0.00));

        // Near 275 Mid 295 Far 315
        public static final Pose2d kCargoSideNearScorePose = new Pose2d(new Translation2d(265.75, -20.5),
                        Rotation2d.fromDegrees(90.00));

        public static final Pose2d kCargoSideMidScorePose = new Pose2d(new Translation2d(285.75, -20.5),
                        Rotation2d.fromDegrees(90.00));

        public static final Pose2d kCargoSideFarScorePose = new Pose2d(new Translation2d(305.75, -20.5),
                        Rotation2d.fromDegrees(90.00));

        public static final Pose2d kCargoNearTurnPose = new Pose2d(new Translation2d(265.75, -75.5),
                        Rotation2d.fromDegrees(90.00));

        public static final Pose2d kCargoMidTurnPose = new Pose2d(new Translation2d(285.75, -75.5),
                        Rotation2d.fromDegrees(90.00));

        public static final Pose2d kCargoFarTurnPose = new Pose2d(new Translation2d(305.75, -75.5),
                        Rotation2d.fromDegrees(90.00));

        public static final Pose2d kCargoMidMidTurnPose = new Pose2d(new Translation2d(265.75, -105.5),
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kCargoNearFaceCargoPose = new Pose2d(new Translation2d(265.75, -60.5),
                        Rotation2d.fromDegrees(90.00));

        public static final Pose2d kCargoMidFaceCargoPose = new Pose2d(new Translation2d(285.75, -60.5),
                        Rotation2d.fromDegrees(90.00));

        public static final Pose2d kCargoFarFaceCargoPose = new Pose2d(new Translation2d(305.75, -60.5),
                        Rotation2d.fromDegrees(90.00));

        public static final Pose2d kCargoNearFaceLoadingPose = new Pose2d(new Translation2d(248.75, -87.5),
                        Rotation2d.fromDegrees(165.00));

        public static final Pose2d kCargoMidFaceLoadingPose = new Pose2d(new Translation2d(283.75, -87.5),
                        Rotation2d.fromDegrees(165.00));

        public static final Pose2d kCargoFarFaceLoadingPose = new Pose2d(new Translation2d(288.75, -87.5),
                        Rotation2d.fromDegrees(165.00));

        public static final Pose2d kLoadingToCargoNear = new Pose2d(new Translation2d(260.75, -60.5),
                        Rotation2d.fromDegrees(185.00));

        public static final Pose2d kLoadingToCargoMid = new Pose2d(new Translation2d(295.75, -60.5),
                        Rotation2d.fromDegrees(185.00));

        public static final Pose2d kLoadingToCargoFar = new Pose2d(new Translation2d(300.75, -60.5),
                        Rotation2d.fromDegrees(185.00));

        public static final Pose2d kCargoSharpTurnNearLoading = new Pose2d(new Translation2d(225.75, -90.5),
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kCargoSharpTurnMidLoading = new Pose2d(new Translation2d(255.75, -90.5),
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kCargoSharpTurnFarLoading = new Pose2d(new Translation2d(275.75, -90.5),
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kCargoToLoadingMidPose = new Pose2d(new Translation2d(250, -108),
                        Rotation2d.fromDegrees(175.00));

        public static final Pose2d kCargoToLoadingReversedMidPose = new Pose2d(new Translation2d(250, -108),
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kLoadingMidPose = new Pose2d(new Translation2d(125, -135),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kLoadingReversedMidPose = new Pose2d(new Translation2d(125, -135),
                        Rotation2d.fromDegrees(0.00));

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
                public final MirroredTrajectory level1StartToRocketBack;
                public final MirroredTrajectory level1StartToCargoSide;
                public final MirroredTrajectory level1StartToCargoFront;
                public final MirroredTrajectory rocketFrontToTurn1A;
                public final MirroredTrajectory rocketFrontTurn1AToLoading;
                public final MirroredTrajectory loadingToRocketBack;
                public final MirroredTrajectory turn3ToRocketBack;
                public final MirroredTrajectory cargoFrontToTurn1;
                public final MirroredTrajectory cargoFrontTurn1ToLoading;
                public final MirroredTrajectory loadingToCargoFrontTrack2v2;
                public final MirroredTrajectory track2v2PoseToCargo2;
                public final MirroredTrajectory cargoNearBackToScorePose;
                public final MirroredTrajectory cargoMidBackToScorePose;
                public final MirroredTrajectory cargoFarBackToScorePose;
                public final MirroredTrajectory cargoFaceNearToBackPose;
                public final MirroredTrajectory cargoFaceMidToBackPose;
                public final MirroredTrajectory cargoFaceFarToBackPose;
                public final MirroredTrajectory cargoBackNearToLoading;
                public final MirroredTrajectory cargoBackMidToLoading;
                public final MirroredTrajectory cargoBackFarToLoading;
                public final MirroredTrajectory cargoSideScoreNearToLoading;
                public final MirroredTrajectory cargoSideScoreMidToLoading;
                public final MirroredTrajectory cargoSideScoreFarToLoading;
                public final MirroredTrajectory loadingToCargoSide;

                public final MirroredTrajectory driveStraight;

                private TrajectorySet() {
                        level1StartToRocketFront = new MirroredTrajectory(getLevel1StartToRocketFront());
                        level1StartToRocketBack = new MirroredTrajectory(getLevel1SideStartToRocketBack());
                        level1StartToCargoSide = new MirroredTrajectory(getLevel1StartToCargoSide());
                        level1StartToCargoFront = new MirroredTrajectory(getLevel1StartToCargoFront());
                        rocketFrontToTurn1A = new MirroredTrajectory(getRocketFrontToTurn1());
                        rocketFrontTurn1AToLoading = new MirroredTrajectory(getRocketFrontTurnToLoading());
                        loadingToRocketBack = new MirroredTrajectory(getLoadingToRocketBack());
                        turn3ToRocketBack = new MirroredTrajectory(getTurn3ToRocketBack());
                        cargoFrontToTurn1 = new MirroredTrajectory(getCargoFrontToTurn1());
                        cargoFrontTurn1ToLoading = new MirroredTrajectory(getCargoFrontTurn1ToLoading());
                        loadingToCargoFrontTrack2v2 = new MirroredTrajectory(getLoadingToCargoFrontTrack2());
                        track2v2PoseToCargo2 = new MirroredTrajectory(getCargoTrack2ToCargoScore2Pose());
                        cargoNearBackToScorePose = new MirroredTrajectory(getCargoFaceNearToScorePose());
                        cargoMidBackToScorePose = new MirroredTrajectory(getCargoFaceMidToScorePose());
                        cargoFarBackToScorePose = new MirroredTrajectory(getCargoFaceFarToScorePose());
                        cargoFaceNearToBackPose = new MirroredTrajectory(getCargoFaceNearToBackPose());
                        cargoFaceMidToBackPose = new MirroredTrajectory(getCargoFaceMidToBackPose());
                        cargoFaceFarToBackPose = new MirroredTrajectory(getCargoFaceFarToBackPose());
                        cargoBackNearToLoading = new MirroredTrajectory(getCargoNearBackToLoading());
                        cargoSideScoreNearToLoading = new MirroredTrajectory(getCargoScoreNearToLoadingPose());
                        cargoSideScoreMidToLoading = new MirroredTrajectory(getCargoScoreMidToLoadingPose());
                        cargoSideScoreFarToLoading = new MirroredTrajectory(getCargoScoreFarToLoadingPose());
                        cargoBackMidToLoading = new MirroredTrajectory(getCargoMidBackToLoading());
                        cargoBackFarToLoading = new MirroredTrajectory(getCargoFarBackToLoading());
                        loadingToCargoSide = new MirroredTrajectory(getLoadingToCargoSide());

                        driveStraight = new MirroredTrajectory(getLevel2StartDriveStraight());

                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2StartDriveStraight() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel2);
                        waypoints.add(kDriveStraightFrontPlatfrom);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1StartToRocketFront() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel1);
                        waypoints.add(kDriveStraightFrontPlatfrom);
                        waypoints.add(kRocketFrontTrackPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1SideStartToRocketBack() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel1Reversed);
                        waypoints.add(kRocketBackTurnPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1StartToCargoSide() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel1);
                        // waypoints.add(kDriveStraightFrontPlatfrom);
                        waypoints.add(kCargoMidMidTurnPose);
                        waypoints.add(kCargoMidTurnPose);
                        waypoints.add(kCargoSideMidScorePose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1StartToCargoFront() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel1);
                        waypoints.add(kDriveStraightFrontPlatfrom);
                        waypoints.add(kCargoFrontPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getRocketFrontToTurn1() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kRocketFrontScorePose);
                        waypoints.add(kRocketFrontTurnPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingToRocketBack() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kLoadingGrabPose);
                        waypoints.add(kMidRocketBackPose);
                        waypoints.add(kRocketBackTurnPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getTurn3ToRocketBack() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kRocketBackTurnPose);
                        waypoints.add(kRocketBackScorePose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFrontToTurn1() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoFrontPose);
                        waypoints.add(kCargoFrontTurn1Pose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFrontTurn1ToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoFrontTurn1Pose);
                        waypoints.add(kLoadingTrackPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getRocketFrontTurnToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kRocketFrontTurnPose);
                        waypoints.add(kLoadingTrackPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingToCargoSide() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kLoadingGrabPose);
                        waypoints.add(kLoadingToCargoNear);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingToCargoFrontTrack2() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kLoadingGrabPose);
                        waypoints.add(kCargo2MidPose);
                        waypoints.add(kCargoFront2TrackPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTrack2ToCargoScore2Pose() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoFront2TrackPose);
                        waypoints.add(kCargoFrontScore2Pose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoNearBackToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoNearFaceLoadingPose);
                        // waypoints.add(kCargoToLoadingMidPose);
                        waypoints.add(kLoadingMidPose);
                        waypoints.add(kLoadingTrackPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, 0, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel,
                                        kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoMidBackToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoMidFaceLoadingPose);
                        // waypoints.add(kCargoToLoadingMidPose);
                        waypoints.add(kLoadingMidPose);
                        waypoints.add(kLoadingTrackPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        96, 0, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFarBackToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoFarFaceLoadingPose);
                        // waypoints.add(kCargoToLoadingMidPose);
                        waypoints.add(kLoadingMidPose);
                        waypoints.add(kLoadingTrackPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFaceNearToBackPose() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoSideNearScorePose);
                        waypoints.add(kCargoNearFaceCargoPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFaceMidToBackPose() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoSideMidScorePose);
                        waypoints.add(kCargoMidFaceCargoPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        0, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel,
                                        kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFaceFarToBackPose() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoSideFarScorePose);
                        waypoints.add(kCargoFarFaceCargoPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        0, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel,
                                        kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFaceNearToScorePose() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoNearFaceCargoPose);
                        waypoints.add(kCargoSideNearScorePose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFaceMidToScorePose() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoMidFaceCargoPose);
                        waypoints.add(kCargoSideMidScorePose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFaceFarToScorePose() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoFarFaceCargoPose);
                        waypoints.add(kCargoSideFarScorePose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoScoreNearToLoadingPose() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoSideNearScorePose);
                        waypoints.add(kLoadingReversedMidPose);
                        waypoints.add(kLoadingReversedTrackPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        0, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel,
                                        kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoScoreMidToLoadingPose() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoSideMidScorePose);
                        waypoints.add(kCargoSharpTurnMidLoading);
                        waypoints.add(kLoadingReversedMidPose);
                        waypoints.add(kLoadingReversedTrackPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        0, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel,
                                        kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoScoreFarToLoadingPose() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoSideFarScorePose);
                        waypoints.add(kCargoSharpTurnFarLoading);
                        waypoints.add(kLoadingReversedMidPose);
                        waypoints.add(kLoadingReversedTrackPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        0, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel,
                                        kMaxVoltage);
                }
        }
}
