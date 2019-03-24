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

        public static final Pose2d kCenterToIntake = new Pose2d(new Translation2d(-18.0, 0.0), Rotation2d.identity());

        public static final Pose2d kSideStartLevel1Intake = kSideStartLevel1.transformBy(kCenterToIntake);

        // STARTING IN CENTER
        public static final Pose2d kCenterStartPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
        public static final Pose2d kSimpleSwitchStartPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
        public static final Pose2d kRightSwitchPose = new Pose2d(new Translation2d(100.0, 0.0),
                        Rotation2d.fromDegrees(0.0));
        public static final Pose2d kLeftSwitchPose = new Pose2d(new Translation2d(10.0, 0.0),
                        Rotation2d.fromDegrees(0.0));

        // 2 Hatch Auto (Front/Back)
        public static final Pose2d kRocketTrackPose = new Pose2d(new Translation2d(183, -131),
                        Rotation2d.fromDegrees(-30.00));
        public static final Pose2d kRocketFrontPose = new Pose2d(new Translation2d(201, -135),
                        Rotation2d.fromDegrees(-30.00));

        public static final Pose2d kRocketFrontToTurn1Pose = new Pose2d(new Translation2d(166, -100),
                        Rotation2d.fromDegrees(-100.00));

        public static final Pose2d kRocketFrontToTurn1APose = new Pose2d(new Translation2d(180, -80),
                        Rotation2d.fromDegrees(-105.00));

        public static final Pose2d kTurn1ToTurn2Pose = new Pose2d(new Translation2d(215, -100),
                        Rotation2d.fromDegrees(-160.00));

                        
        public static final Pose2d kTurn2ToLoadingMidPose = new Pose2d(new Translation2d(120, -135),
        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kTurn1ToLoadingPose = new Pose2d(new Translation2d(30, -135),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kLoadingPose = new Pose2d(new Translation2d(30.00, -130),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kLoadingGrabPose = new Pose2d(new Translation2d(0.00, -130),
                        Rotation2d.fromDegrees(-180.0));

        // public static final Pose2d kLoadingToMidRocketBack = new Pose2d(new
        // Translation2d(220, -110),
        // Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kLoadingToMidRocketBack = new Pose2d(new Translation2d(230, -110),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kRocketBackPoseToTurnPose = new Pose2d(new
        Translation2d(260, -80),
        Rotation2d.fromDegrees(-120.00));

        
        public static final Pose2d kRocketBackRightPoseToTurnPose = new Pose2d(new
        Translation2d(260, -80),
        Rotation2d.fromDegrees(-120.00));

        // public static final Pose2d kRocketBackPoseToTurnPose = new Pose2d(new
        // Translation2d(310, -130),
        // Rotation2d.fromDegrees(160.00));

        // public static final Pose2d kRocketBackPoseToTurnPose = new Pose2d(new Translation2d(275, -110),
        //                 Rotation2d.fromDegrees(-147.00));

        // public static final Pose2d kRocketBackPose = new Pose2d(new Translation2d(270, -140),
        //                 Rotation2d.fromDegrees(-150.00));

        public static final Pose2d kRocketBackPose = new Pose2d(new
        Translation2d(258, -138),
        Rotation2d.fromDegrees(-150.00));

        // Cargo 2 Hatch (Middle/Side)

        public static final Pose2d kCargoTrackPoseSideStart = new Pose2d(new Translation2d(195, -14),
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kCargoFrontPose = new Pose2d(new Translation2d(215, -14), //207.7
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kCargoFrontToTurn1Pose = new Pose2d(new Translation2d(175, -90),
                        Rotation2d.fromDegrees(90.00));

        public static final Pose2d kCargoFrontToTurn1v2Pose = new Pose2d(new Translation2d(175, -90),
                        Rotation2d.fromDegrees(-90.00));

        public static final Pose2d kCargoFrontToTurn1APose = new Pose2d(new Translation2d(180, -90),
                        Rotation2d.fromDegrees(135.00));

        public static final Pose2d kTurn1CargoFrontToLoadingPose = new Pose2d(new Translation2d(30, -130),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kCargoSide1Pose = new Pose2d(new Translation2d(290.75, -20.5), // Near 275 Mid 295 Far 320 50 Front of Cargo
                        Rotation2d.fromDegrees(90.00));

        public static final Pose2d kCargoSideTurnPose = new Pose2d(new Translation2d(290.75, -60.5), 
                        Rotation2d.fromDegrees(90.00));

        public static final Pose2d kCargoSideTurnToLoadingPose = new Pose2d(new Translation2d(295.75, -60.5), 
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kLoadingToCargoFront2Pose = new Pose2d(new Translation2d(120, 20),
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kLoadingToCargoFrontMidPose= new Pose2d(new Translation2d(185, 0), //200
                        Rotation2d.fromDegrees(-90.00));

        public static final Pose2d kMidPoseToCargoFront2v2Pose = new Pose2d(new Translation2d(155, 14), //170
                        Rotation2d.fromDegrees(0.00));



        public static final Pose2d kCargoTurn1LoadingToTrackPose = new Pose2d(new Translation2d(230, 14), //207
                        Rotation2d.fromDegrees(0.00));

        public static final Pose2d kTurn1CargoSide1Pose = new Pose2d(new Translation2d(295, -115),
                        Rotation2d.fromDegrees(135.00));

        public static final Pose2d kCargoFrontTurnPose = new Pose2d(new Translation2d(200, -13),
                        Rotation2d.fromDegrees(-90.00));

        public static final Pose2d kTurnPoseToMidPose = new Pose2d(new Translation2d(180, -60),
                        Rotation2d.fromDegrees(-100.00));

        public static final Pose2d kCargoFrontToLoadingPose = new Pose2d(new Translation2d(30, -130),
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

                public final MirroredTrajectory centerStartToLeftSwitch;
                public final MirroredTrajectory centerStartToRightSwitch;
                public final MirroredTrajectory simpleStartToLeftSwitch;
                public final MirroredTrajectory simpleStartToRightSwitch;
                public final MirroredTrajectory level1StartToRocketFront;
                public final MirroredTrajectory level1StartToRocketBack;
                public final MirroredTrajectory level1StartToCargoSideTurn;
                public final MirroredTrajectory loadingToRocketBack;
                public final MirroredTrajectory loadingToRocketBackRight;
                public final MirroredTrajectory rocketFrontToTurn1;
                public final MirroredTrajectory rocketFrontToTurn1A;
                public final MirroredTrajectory turn1ToTurn2;
                public final MirroredTrajectory turn2ToLoading;
                public final MirroredTrajectory turn3ToRocketBack;
                public final MirroredTrajectory turn3ToRocketBackRight;
                public final MirroredTrajectory level1StartToCargoFront;
                public final MirroredTrajectory cargoFrontToTurn1;
                public final MirroredTrajectory cargoFrontToTurn1A;
                public final MirroredTrajectory cargoFrontToTurn1v2;
                public final MirroredTrajectory cargoFrontToLoading;
                public final MirroredTrajectory cargoFrontTurn1ToLoading;
                public final MirroredTrajectory cargoFrontTurn1v2ToLoading;
                public final MirroredTrajectory cargoFrontTurn1AToLoading;
                public final MirroredTrajectory cargoSideScoreToLoading;
                public final MirroredTrajectory rocketFrontTurn1AToLoading;
                public final MirroredTrajectory loadingToTurn1CargoSide1;
                public final MirroredTrajectory turn1ToCargoSide1;
                public final MirroredTrajectory turnSideCargoToSideCargoScore;
                public final MirroredTrajectory loadingTocargoFrontTrack2;
                public final MirroredTrajectory loadingToCargoFrontTrack2v2;
                public final MirroredTrajectory track2PoseToCargo2;
                public final MirroredTrajectory track2v2PoseToCargo2;
                
                private TrajectorySet() {
                        level1StartToRocketFront = new MirroredTrajectory(getLevel1StartToRocketFront());
                        level1StartToRocketBack = new MirroredTrajectory(getLevel1SideStartToRocketBack());
                        level1StartToCargoSideTurn = new MirroredTrajectory(getLevel1StartToCargoSideTurn());
                        level1StartToCargoFront = new MirroredTrajectory(getLevel1StartToCargoFront());
                        rocketFrontToTurn1 = new MirroredTrajectory(getRocketFrontToTurn1());
                        rocketFrontToTurn1A = new MirroredTrajectory(getRocketFrontToTurn1A());
                        turn1ToTurn2 = new MirroredTrajectory(getTurn1ToTurn2());
                        turn2ToLoading = new MirroredTrajectory(getTurn2ToLoading());
                        turn3ToRocketBack = new MirroredTrajectory(getTurn3ToRocketBack());
                        turn3ToRocketBackRight = new MirroredTrajectory(getTurn3ToRocketBackRight());
                        loadingToRocketBack = new MirroredTrajectory(getLoadingToRocketBack());
                        loadingToRocketBackRight = new MirroredTrajectory(getLoadingToRocketBackRight());
                        cargoFrontToTurn1 = new MirroredTrajectory(getCargoFrontToTurn1());
                        cargoFrontToTurn1A = new MirroredTrajectory(getCargoFrontToTurn1A());
                        cargoFrontToTurn1v2 = new MirroredTrajectory(getCargoFrontToTurn1v2());
                        cargoFrontToLoading = new MirroredTrajectory(getCargoFrontToLoading());
                        cargoFrontTurn1ToLoading = new MirroredTrajectory(getCargoFrontTurn1ToLoading());
                        cargoFrontTurn1v2ToLoading = new MirroredTrajectory(getCargoFrontTurn1v2ToLoading());
                        cargoFrontTurn1AToLoading = new MirroredTrajectory(getCargoFrontTurn1AToLoading());
                        cargoSideScoreToLoading = new MirroredTrajectory(getCargoSideScoreToLoading());
                        rocketFrontTurn1AToLoading = new MirroredTrajectory(getRocketFrontTurn1AToLoading());
                        loadingToTurn1CargoSide1 = new MirroredTrajectory(getLoadingToTurn1CargoSide1());
                        turn1ToCargoSide1 = new MirroredTrajectory(getTurn1ToCargoSide1());
                        turnSideCargoToSideCargoScore = new MirroredTrajectory(getCargoSideTurnToSideCargoScore());
                        loadingTocargoFrontTrack2 = new MirroredTrajectory(getLoadingToCargoFrontTrack2());
                        loadingToCargoFrontTrack2v2 = new MirroredTrajectory(getLoadingToCargoFrontTrack2v2());
                        track2PoseToCargo2 = new MirroredTrajectory(getCargoTrack2ToCargocScorePose());
                        track2v2PoseToCargo2 = new MirroredTrajectory(getCargoTrack2v2ToCargoScorePose());

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

                private Trajectory<TimedState<Pose2dWithCurvature>> getRocketFrontToTurn1() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kRocketFrontPose);
                        waypoints.add(kRocketFrontToTurn1Pose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getRocketFrontToTurn1A() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kRocketFrontPose);
                        waypoints.add(kRocketFrontToTurn1APose);

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
                        waypoints.add(kTurn2ToLoadingMidPose);
                        waypoints.add(kTurn1ToLoadingPose);

                        return generateTrajectory(false, waypoints,
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

                private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingToRocketBackRight() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kLoadingGrabPose);
                        waypoints.add(kLoadingToMidRocketBack);
                        waypoints.add(kRocketBackRightPoseToTurnPose);

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

                private Trajectory<TimedState<Pose2dWithCurvature>> getTurn3ToRocketBackRight() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kRocketBackRightPoseToTurnPose);
                        waypoints.add(kRocketBackPose);

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

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFrontToTurn1() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoFrontPose);
                        waypoints.add(kCargoFrontToTurn1Pose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFrontTurn1ToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoFrontToTurn1Pose);
                        waypoints.add(kLoadingPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getRocketFrontTurn1AToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kRocketFrontToTurn1APose);
                        waypoints.add(kLoadingPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingToTurn1CargoSide1() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kLoadingPose);
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

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFrontToTurn1A() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoFrontPose);
                        waypoints.add(kCargoFrontToTurn1APose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFrontToTurn1v2() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoFrontPose);
                        waypoints.add(kCargoFrontToTurn1v2Pose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                
                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFrontToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoFrontTurnPose);
                        waypoints.add(kTurnPoseToMidPose);
                        waypoints.add(kCargoFrontToLoadingPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFrontTurn1AToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoFrontToTurn1APose);
                        waypoints.add(kLoadingPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFrontTurn1v2ToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoFrontToTurn1v2Pose);
                        waypoints.add(kLoadingPose);

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

                private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingToCargoFrontTrack2v2() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kLoadingGrabPose);
                        waypoints.add(kLoadingToCargoFrontMidPose);
                        waypoints.add(kMidPoseToCargoFront2v2Pose);

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

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTrack2v2ToCargoScorePose() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kMidPoseToCargoFront2v2Pose);
                        waypoints.add(kCargoTurn1LoadingToTrackPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1SideStartToRocketBack() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel1);
                        waypoints.add(kRocketBackPoseToTurnPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1StartToCargoSideTurn() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel1);
                        waypoints.add(kCargoSideTurnPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoSideTurnToSideCargoScore() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoSideTurnPose);
                        waypoints.add(kCargoSide1Pose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoSideScoreToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoSideTurnToLoadingPose);
                        waypoints.add(kLoadingPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }
        }
}
