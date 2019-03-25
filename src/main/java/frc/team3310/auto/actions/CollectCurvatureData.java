package frc.team3310.auto.actions;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import frc.team3310.robot.subsystems.Drive;
import frc.team3310.utility.DriveSignal;
import frc.team3310.utility.ReflectingCSVWriter;
import frc.team3310.utility.lib.control.RobotStatus;
import frc.team3310.utility.lib.physics.DriveCharacterization;

public class CollectCurvatureData implements Action {
    private static final double kMaxPower = 0.4;
    private static final double kStartPower = 0.2;
    private static final double kStartTime = 0.25;
    private static final double kRampRate = 0.02;
    private static final Drive mDrive = Drive.getInstance();
    private static final RobotStatus mRobotState = RobotStatus.getInstance();

    private final ReflectingCSVWriter<DriveCharacterization.CurvatureDataPoint> mCSVWriter;
    private final List<DriveCharacterization.CurvatureDataPoint> mCurvatureData;
    private final boolean mReverse;
    private final boolean mHighGear;

    private boolean isFinished = false;
    private double mStartTime = 0.0;

    /**
     * @param data     reference to the list where data points should be stored
     * @param highGear use high gear or low
     * @param reverse  if true drive in reverse, if false drive normally
     */

    public CollectCurvatureData(List<DriveCharacterization.CurvatureDataPoint> data, boolean highGear, boolean reverse) {
        mCurvatureData = data;
        mHighGear = highGear;
        mReverse = reverse;
        mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/CURVATURE_DATA.csv", DriveCharacterization.CurvatureDataPoint.class);

    }

    @Override
    public void start() {
        mDrive.setOpenLoop(new DriveSignal(kStartPower, kStartPower));
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        double t = Timer.getFPGATimestamp() - mStartTime;
        if (t < kStartTime) { //give the robot some time to accelerate before recording data
            return;
        }
        double rightPower = kStartPower + (t - kStartTime) * kRampRate;
        if (rightPower > kMaxPower) {
            isFinished = true;
            return;
        }
        mDrive.setOpenLoop(new DriveSignal((mReverse ? -1.0 : 1.0) * kStartPower, (mReverse ? -1.0 : 1.0) * rightPower));
        mCurvatureData.add(new DriveCharacterization.CurvatureDataPoint(
                mRobotState.getMeasuredVelocity().dx,
                mRobotState.getMeasuredVelocity().dtheta,
                kStartPower,
                rightPower
        ));
        mCSVWriter.add(mCurvatureData.get(mCurvatureData.size() - 1));
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void done() {
        mDrive.setOpenLoop(DriveSignal.BRAKE);
        mCSVWriter.flush();
    }
}
