package com.team3310.frc2019.paths;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.team3310.robot.paths.TrajectoryGenerator;
import frc.team3310.utility.Util;
import frc.team3310.utility.lib.geometry.Pose2d;
import frc.team3310.utility.lib.geometry.Pose2dWithCurvature;
import frc.team3310.utility.lib.geometry.Twist2d;
import frc.team3310.utility.lib.trajectory.LazyLoadTrajectory;
import frc.team3310.utility.lib.trajectory.MirroredTrajectory;
import frc.team3310.utility.lib.trajectory.TimedView;
import frc.team3310.utility.lib.trajectory.TrajectoryIterator;
import frc.team3310.utility.lib.trajectory.timing.TimedState;

public class TrajectoryGeneratorTest {
        public static final double kTestEpsilon = 1e-5;

        public void verifyMirroredTrajectories(final MirroredTrajectory mirrored, boolean shouldBeReversed) {
                assertEquals(mirrored.left.length(), mirrored.right.length());
                TrajectoryIterator<TimedState<Pose2dWithCurvature>> left_iterator = new TrajectoryIterator<>(
                                new TimedView<>(mirrored.left));
                TrajectoryIterator<TimedState<Pose2dWithCurvature>> right_iterator = new TrajectoryIterator<>(
                                new TimedView<>(mirrored.right));

                final double dt = 0.05;
                TimedState<Pose2dWithCurvature> prev_left = null;
                TimedState<Pose2dWithCurvature> prev_right = null;
                while (!left_iterator.isDone() && !right_iterator.isDone()) {
                        TimedState<Pose2dWithCurvature> left_state = left_iterator.getState();
                        TimedState<Pose2dWithCurvature> right_state = right_iterator.getState();

                        assertEquals(left_state.t(), right_state.t(), kTestEpsilon);
                        assertEquals(left_state.velocity(), right_state.velocity(), kTestEpsilon);
                        assertEquals(left_state.acceleration(), right_state.acceleration(), kTestEpsilon);

                        assertTrue((shouldBeReversed ? -1.0 : 1.0) * left_state.velocity() >= -kTestEpsilon);
                        assertTrue((shouldBeReversed ? -1.0 : 1.0) * right_state.velocity() >= -kTestEpsilon);

                        if (prev_left != null && prev_right != null) {
                                // Check there are no angle discontinuities.
                                final double kMaxReasonableChangeInAngle = 0.3; // rad
                                Twist2d left_change = Pose2d.log(prev_left.state().getPose().inverse()
                                                .transformBy(left_state.state().getPose()));
                                Twist2d right_change = Pose2d.log(prev_right.state().getPose().inverse()
                                                .transformBy(right_state.state().getPose()));
                                assertTrue(Math.abs(left_change.dtheta) < kMaxReasonableChangeInAngle);
                                assertTrue(Math.abs(right_change.dtheta) < kMaxReasonableChangeInAngle);
                                if (!Util.epsilonEquals(left_change.dtheta, 0.0)
                                                || !Util.epsilonEquals(right_change.dtheta, 0.0)) {
                                        // Could be a curvature sign change between prev and now, so just check that
                                        // either matches our
                                        // expected sign.
                                        final boolean left_curvature_positive = left_state.state()
                                                        .getCurvature() > kTestEpsilon
                                                        || prev_left.state().getCurvature() > kTestEpsilon;
                                        final boolean left_curvature_negative = left_state.state()
                                                        .getCurvature() < -kTestEpsilon
                                                        || prev_left.state().getCurvature() < -kTestEpsilon;
                                        final boolean right_curvature_positive = right_state.state()
                                                        .getCurvature() > kTestEpsilon
                                                        || prev_right.state().getCurvature() > kTestEpsilon;
                                        final boolean right_curvature_negative = right_state.state()
                                                        .getCurvature() < -kTestEpsilon
                                                        || prev_right.state().getCurvature() < -kTestEpsilon;
                                        final double actual_left_curvature = left_change.dtheta / left_change.dx;
                                        final double actual_right_curvature = right_change.dtheta / right_change.dx;
                                        if (actual_left_curvature < -kTestEpsilon) {
                                                System.out.println("Left < curvature = " + actual_left_curvature);
 //                                               assertTrue(left_curvature_negative);
                                        } else if (actual_left_curvature > kTestEpsilon) {
                                                System.out.println("Left > curvature = " + actual_left_curvature);
//                                               assertTrue(left_curvature_positive);
                                        }
                                        if (actual_right_curvature < -kTestEpsilon) {
                                                System.out.println("Right < curvature = " + actual_left_curvature);
//                                               assertTrue(right_curvature_negative);
                                        } else if (actual_right_curvature > kTestEpsilon) {
                                                System.out.println("Right > curvature = " + actual_left_curvature);
//                                               assertTrue(right_curvature_positive);
                                        }
                                }
                        }

                        assertEquals(left_state.state().getTranslation().x(), right_state.state().getTranslation().x(),
                                        Util.kEpsilon);
                        assertEquals(left_state.state().getTranslation().y(), -right_state.state().getTranslation().y(),
                                        Util.kEpsilon);
                        assertEquals(left_state.state().getRotation(), right_state.state().getRotation().inverse());
                        assertEquals(left_state.state().getCurvature(), -right_state.state().getCurvature(),
                                        kTestEpsilon);

                        left_iterator.advance(dt);
                        right_iterator.advance(dt);
                        prev_left = left_state;
                        prev_right = right_state;
                }
                assertTrue(left_iterator.isDone() && right_iterator.isDone());
        }

        @Test
        public void test() {
                TrajectoryGenerator.getInstance().generateTrajectories();

                LazyLoadTrajectory level1StartToRocketFront = TrajectoryGenerator.getInstance().getTrajectorySet().level1StartToRocketFront;
                level1StartToRocketFront.activate();
                verifyMirroredTrajectories(level1StartToRocketFront.getTrajectory(), false);

                LazyLoadTrajectory loadingToRocketBack = TrajectoryGenerator.getInstance().getTrajectorySet().loadingToRocketBack;
                loadingToRocketBack.activate();
                verifyMirroredTrajectories(loadingToRocketBack.getTrajectory(), true);

                // verifyMirroredTrajectories(TrajectoryGenerator.getInstance().getTrajectorySet().rocketFrontToLoading,
                // true);

        }
}
