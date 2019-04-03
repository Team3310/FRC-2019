package frc.team3310.robot.paths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.team3310.robot.planners.DriveMotionPlanner;
import frc.team3310.utility.lib.geometry.Pose2d;
import frc.team3310.utility.lib.geometry.Pose2dWithCurvature;
import frc.team3310.utility.lib.geometry.Rotation2d;
import frc.team3310.utility.lib.geometry.Translation2d;
import frc.team3310.utility.lib.trajectory.LazyLoadTrajectory;
import frc.team3310.utility.lib.trajectory.Trajectory;
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

        public static final Pose2d kSideStartLevel2 = new Pose2d(42.118, -47.4, Rotation2d.fromDegrees(0.0));

        public static final Pose2d kStartReversedLevel2 = new Pose2d(42.25, -47.4, Rotation2d.fromDegrees(-180.0));

        public static final Pose2d kFrontPlatfromPose = new Pose2d(new Translation2d(100, -48.4),
                        Rotation2d.fromDegrees(0.0));

        public static final Pose2d kReversedFrontPlatfromPose = new Pose2d(new Translation2d(100, -47.4),
                        Rotation2d.fromDegrees(-180.0));

        // Multipurposes Poses
        public static final Pose2d kLoadingTrackPose = new Pose2d(new Translation2d(30.00, -135),
                        Rotation2d.fromDegrees(180.00));

        public static final Pose2d kLoadingGrabPose = new Pose2d(new Translation2d(0.00, -135),
                        Rotation2d.fromDegrees(180.0));

        public static final Pose2d kLoadingMidPose = new Pose2d(new Translation2d(125, -135),
                        Rotation2d.fromDegrees(180.00));


        public static final Pose2d kLoadingMidFudgePose = new Pose2d(new Translation2d(125, -130),
                        Rotation2d.fromDegrees(180.00));

        public static final Pose2d kLoadingMidFudgev2Pose = new Pose2d(new Translation2d(125, -140),
                        Rotation2d.fromDegrees(180.00));

        public static final Pose2d kLoadingGrabFudgePose = new Pose2d(new Translation2d(0, -130),
                        Rotation2d.fromDegrees(180.00));

        public static final Pose2d kLoadingGrabFudgev2Pose = new Pose2d(new Translation2d(0, -140),
                        Rotation2d.fromDegrees(180.00));

                        

        // Rocket Poses
        // Front
        public static final Pose2d kRocketFrontTrackPose = new Pose2d(new Translation2d(183, -127),
                        Rotation2d.fromDegrees(-30.00));
        public static final Pose2d kRocketFrontScorePose = new Pose2d(new Translation2d(201, -135),
                        Rotation2d.fromDegrees(-30.00));

        public static final Pose2d kRocketFrontTurnPose = new Pose2d(new Translation2d(180, -80),
                        Rotation2d.fromDegrees(-105.00));
        
        // Back
         public static final Pose2d kRocketBackPose = new Pose2d(new Translation2d(290, -135),
                        Rotation2d.fromDegrees(135.00)); // 135 = -45 facing feild

        public static final Pose2d kMissRocketBack = new Pose2d(new Translation2d(250, -100),
                        Rotation2d.fromDegrees(150.00)); // 135 = -45 facing feild

        public static final Pose2d kRocketBackTurnPose = new Pose2d(new Translation2d(290, -125),
                Rotation2d.fromDegrees(160.00));

        public static final Pose2d kRocketBackScorePose = new Pose2d(new Translation2d(258, -138),
                 Rotation2d.fromDegrees(-150.00)); // -150 = 30 facing wall

        public static final Pose2d kMidRocketBackPose = new Pose2d(new Translation2d(230, -100),
                  Rotation2d.fromDegrees(180.00));

        // Cargo Poses
        // Front
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

        // Side
        // Near 256 Mid 280 Far 295

        public static final Pose2d kCargoNearReversedPose = new Pose2d(new Translation2d(256.75, -80.5),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kCargoMidReversedPose = new Pose2d(new Translation2d(280.75, -80.5),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kCargoFarReversedPose = new Pose2d(new Translation2d(304.75, -80.5),
                        Rotation2d.fromDegrees(-180.00));

        public static final Pose2d kCargoNearFaceLoadingPose = new Pose2d(new Translation2d(239.75, -67.5),
                        Rotation2d.fromDegrees(165.00));

        public static final Pose2d kCargoMidFaceLoadingPose = new Pose2d(new Translation2d(263.75, -67.5),
                        Rotation2d.fromDegrees(-160.00));

        public static final Pose2d kCargoFarFaceLoadingPose = new Pose2d(new Translation2d(287.75, -67.5),
                        Rotation2d.fromDegrees(165.00));

        public static final Pose2d kLoadingToCargoNear = new Pose2d(new Translation2d(256.75, -80.5),
                        Rotation2d.fromDegrees(181.00));

        public static final Pose2d kLoadingToCargoMid = new Pose2d(new Translation2d(280.75, -60.5),
                        Rotation2d.fromDegrees(181.00));

        public static final Pose2d kLoadingToCargoFar = new Pose2d(new Translation2d(304.75, -60.5),
                        Rotation2d.fromDegrees(181.00));

        public class TrajectorySet {

                public final LazyLoadTrajectory platformToCargoSide;
                public final LazyLoadTrajectory level1StartToRocketFront;
                public final LazyLoadTrajectory level1StartReversedToRocketBack;
                public final LazyLoadTrajectory level1StartReversedToCargoSide;
                public final LazyLoadTrajectory level1StartToCargoFront;
                public final LazyLoadTrajectory rocketFrontToTurn1A;
                public final LazyLoadTrajectory rocketFrontTurn1AToLoading;
                public final LazyLoadTrajectory rocketBackToLoading;
                public final LazyLoadTrajectory loadingToRocketBack;
                public final LazyLoadTrajectory turn3ToRocketBack;
                public final LazyLoadTrajectory cargoFrontToTurn1;
                public final LazyLoadTrajectory cargoFrontTurn1ToLoading;
                public final LazyLoadTrajectory loadingToCargoFrontTrack2v2;
                public final LazyLoadTrajectory track2v2PoseToCargo2;
                public final LazyLoadTrajectory cargoBackNearToLoading;
                public final LazyLoadTrajectory cargoBackMidToLoading;
                public final LazyLoadTrajectory cargoBackFarToLoading;
                public final LazyLoadTrajectory loadingToCargoSide;

                public final LazyLoadTrajectory driveStraight;

                private TrajectorySet() {
                        platformToCargoSide             = new LazyLoadTrajectory(()->getPlatformToCargoSideReversed());
                        level1StartToRocketFront        = new LazyLoadTrajectory(()->getLevel1StartToRocketFront());
                        level1StartReversedToRocketBack = new LazyLoadTrajectory(()->getLevel1SideStartToRocketBack());
                        level1StartReversedToCargoSide  = new LazyLoadTrajectory(()->getLevel1StartToCargoSideReversed());
                        level1StartToCargoFront         = new LazyLoadTrajectory(()->getLevel1StartToCargoFront());
                        rocketFrontToTurn1A             = new LazyLoadTrajectory(()->getRocketFrontToTurn1());
                        rocketFrontTurn1AToLoading      = new LazyLoadTrajectory(()->getRocketFrontTurnToLoading());
                        rocketBackToLoading             = new LazyLoadTrajectory(()->getRocketBackToLoading());
                        loadingToRocketBack             = new LazyLoadTrajectory(()->getLoadingToRocketBack());
                        turn3ToRocketBack               = new LazyLoadTrajectory(()->getTurn3ToRocketBack());
                        cargoFrontToTurn1               = new LazyLoadTrajectory(()->getCargoFrontToTurn1());
                        cargoFrontTurn1ToLoading        = new LazyLoadTrajectory(()->getCargoFrontTurn1ToLoading());
                        loadingToCargoFrontTrack2v2     = new LazyLoadTrajectory(()->getLoadingToCargoFrontTrack2());
                        track2v2PoseToCargo2            = new LazyLoadTrajectory(()->getCargoTrack2ToCargoScore2Pose());
                        cargoBackNearToLoading          = new LazyLoadTrajectory(()->getCargoNearBackToLoading());
                        cargoBackMidToLoading           = new LazyLoadTrajectory(()->getCargoMidBackToLoading());
                        cargoBackFarToLoading           = new LazyLoadTrajectory(()->getCargoFarBackToLoading());
                        loadingToCargoSide              = new LazyLoadTrajectory(()->getLoadingToCargoSide());

                        driveStraight                   = new LazyLoadTrajectory(()->getLevel2StartDriveStraight());
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getPlatformToCargoSideReversed() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kReversedFrontPlatfromPose);
                        waypoints.add(kCargoMidReversedPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, 0, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel,
                                        kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2StartDriveStraight() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kStartReversedLevel2);
                        waypoints.add(kReversedFrontPlatfromPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kSimpleSwitchMaxCentripetalAccel)),
                                        0, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel,
                                        kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1StartToRocketFront() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel1);
                        waypoints.add(kFrontPlatfromPose);
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

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1StartToCargoSideReversed() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel1Reversed);
                        waypoints.add(kCargoMidReversedPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1StartToCargoFront() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kSideStartLevel1);
                        waypoints.add(kFrontPlatfromPose);
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

                private Trajectory<TimedState<Pose2dWithCurvature>> getRocketBackToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kRocketBackPose);
                        waypoints.add(kMissRocketBack);
                        waypoints.add(kLoadingMidFudgePose);
                        waypoints.add(kLoadingGrabFudgePose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
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
                        waypoints.add(kLoadingGrabFudgePose);

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

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFarBackToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoFarFaceLoadingPose);
                        waypoints.add(kLoadingMidFudgev2Pose);
                        waypoints.add(kLoadingGrabFudgev2Pose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        96, 0, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoMidBackToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoMidFaceLoadingPose);
                        waypoints.add(kLoadingMidFudgev2Pose);
                        waypoints.add(kLoadingGrabFudgev2Pose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        96, 0, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }
                
                private Trajectory<TimedState<Pose2dWithCurvature>> getCargoNearBackToLoading() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kCargoNearFaceLoadingPose);
                        waypoints.add(kLoadingMidFudgev2Pose);
                        waypoints.add(kLoadingGrabFudgev2Pose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(
                                                        kMaxCentripetalAccelElevatorDown)),
                                        96, 0, kSimpleSwitchMaxVelocity, kSimpleSwitchMaxAccel, kMaxVoltage);
                }
        }
}
