package com.spartronics4915.frc2024.subsystems.vision;

import java.util.Optional;

import com.spartronics4915.frc2024.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightDevice extends SubsystemBase {

    public static record VisionMeasurement(Pose3d pose, double timestamp) {}

    private final String mName;
    private final Field2d mField;
    private final boolean mValid;

    BooleanPublisher controllablePub;
    StringPublisher typePub;
    DoublePublisher xPub;
    DoublePublisher yPub;
    DoublePublisher zPub;

    /**
     * Creates a new LimelightDevice
     * @param name The name of the limelight
     */
    public LimelightDevice(String name) {
        String formattedName = "limelight-" + name;
        mName = formattedName;
        mValid = !NetworkTableInstance.getDefault().getTable(formattedName).getKeys().isEmpty();
        mField = new Field2d();
        createShuffleboard();
        fakeAccelerometer();
    }

    public Optional<VisionMeasurement> getVisionMeasurement() {
        if (!canSeeTags()) {
            return Optional.empty();
        }
        double[] botpose = LimelightHelpers.getBotPose_wpiBlue(mName);
        Pose3d pose = new Pose3d(botpose[0], botpose[1], botpose[2], new Rotation3d(botpose[3], botpose[4], botpose[5]));
        double timestamp = Timer.getFPGATimestamp() - (botpose[6]/1000.0);
        return Optional.of(new VisionMeasurement(pose, timestamp));
    }
    
    /**
     * @return If the limelight can see any tags
     */
    public boolean canSeeTags() {
        if (!mValid) return false;
        return LimelightHelpers.getTV(mName);
    }

    /**
     * @return The {@link Pose3d} of the robot, as estimated from Limelight MetaTag
     */
    public Pose3d getBotPose3d() {
        if (!mValid) return new Pose3d();
        return LimelightHelpers.getBotPose3d(mName);
    }

    /**
     * @return The {@link Pose2d} of the robot, as estimated from Limelight MetaTag
     */
    public Pose2d getBotPose2d() {
        if (!mValid) return new Pose2d();
        return LimelightHelpers.getBotPose2d(mName);
    }

    /**
     * @return The number of tags seen by the limelight
     */
    public int numberOfTagsSeen() {
        if (!mValid) return 0;
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults(mName);
        LimelightHelpers.LimelightTarget_Fiducial[] fiducials = llresults.targetingResults.targets_Fiducials;
        int tagCount = fiducials.length;
        return tagCount;
    }

    /**
     * @return The current pipeline index of the limelight
     */
    public int getCurrentPipelineIndex() {
        if (!mValid) return 0;
        return (int) LimelightHelpers.getCurrentPipelineIndex(mName);
    }
    /**
     * Sets the current pipeline index to the value provided
     * @param index The pipeline index between 0-9
     */
    public void setCurrentPipelineIndex(int index) {
        LimelightHelpers.setPipelineIndex(mName, index);
    }

    /**
     * @return The id of the primary in-view tag, or 0 if there are none
     */
    public int getPrimaryTag() {
        if (!mValid) return 0;
        return (int) LimelightHelpers.getFiducialID(mName);

    }

    /**
     * @param tag
     * @return The distance to the tag
     */
    private double distanceToTag(LimelightHelpers.LimelightTarget_Fiducial tag) {
        Pose3d pose = tag.getTargetPose_CameraSpace(); //this should be the right method
        return Math.sqrt(pose.getX() * pose.getX() + pose.getY() * pose.getY() + pose.getZ() * pose.getZ());
    }

    /**
     * @return The average distance to the visible tags, or 0 if none are seen
     */
    public double getAverageDistanceToVisibleTags() {
        if (!mValid) return 0.0;
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults(mName);
        LimelightHelpers.LimelightTarget_Fiducial[] fiducials = llresults.targetingResults.targets_Fiducials;
        double averageDistance = 0.0;
        for(LimelightHelpers.LimelightTarget_Fiducial tag : fiducials) {
            averageDistance += distanceToTag(tag);
        }
        averageDistance /= fiducials.length;
        return averageDistance;
    }

    /**
     * @return The name of the limelight
     */
    public String getmName() {
        return mName;
    }

    public void updateFieldPose() {
        mField.setRobotPose(getBotPose2d());
    }

    public double[] getXYZOfPrimaryTag() {
        if (!mValid) return new double[3];
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults(mName);
        LimelightHelpers.LimelightTarget_Fiducial[] fiducials = llresults.targetingResults.targets_Fiducials;
        LimelightHelpers.LimelightTarget_Fiducial primaryTag = null;
        for(LimelightHelpers.LimelightTarget_Fiducial tag : fiducials) {
            if (tag.fiducialID == LimelightHelpers.getFiducialID(mName)) primaryTag = tag;
        }
        double[] xyz = new double[3];
        xyz[0] = 0.0;
        xyz[1] = 0.0;
        xyz[2] = 0.0;
        if (primaryTag == null) return xyz;
        Pose3d pose = primaryTag.getCameraPose_TargetSpace();
        xyz[0] = pose.getX();
        xyz[1] = pose.getY();
        xyz[2] = pose.getZ();
        return xyz;
    }

    public void fakeAccelerometer() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Shuffleboard").getSubTable("VisionTab").getSubTable(mName + " position");

    controllablePub = table.getBooleanTopic(".controllable").publish();
    typePub = table.getStringTopic(".type").publish();
    xPub = table.getDoubleTopic("X").publish();
    yPub = table.getDoubleTopic("Y").publish();
    zPub = table.getDoubleTopic("Z").publish();

    controllablePub.set(true);
    typePub.set("3AxisAccelerometer");
    xPub.set(0.0);
    yPub.set(1.0);
    zPub.set(2.0);
    }

    public void updateAccelerometer(double x, double y, double z) {
        xPub.set(x);
        yPub.set(y);
        zPub.set(z);
    }

    public void updateAccelerometer() {
        double[] xyz = getXYZOfPrimaryTag();
        updateAccelerometer(xyz[0], xyz[1], xyz[2]);
    }

    public void createShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("VisionTab");
        final int offset = (mName == "limelight-bob" ? 5 : 0);
        // tab.addInteger(mName + " primary tag", () -> {return getPrimaryTag();}).withPosition(0 + offset, 3);
        tab.addBoolean(mName + " is valid", () -> {return mValid;}).withPosition(0 + offset, 3);
        tab.addInteger(mName + " tag count", () -> {return numberOfTagsSeen();}).withPosition(1 + offset, 3);
        tab.addBoolean(mName + " sees tag", () -> {return canSeeTags();}).withPosition(2 + offset, 3);
        tab.addDouble(mName + " avg dist", () -> {return getAverageDistanceToVisibleTags();}).withPosition(3 + offset, 3);
        tab.add(mName + " field", mField).withSize(5, 3).withPosition(0 + offset, 0);
    }
}
