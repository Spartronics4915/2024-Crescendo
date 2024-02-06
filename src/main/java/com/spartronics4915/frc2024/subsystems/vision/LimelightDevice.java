package com.spartronics4915.frc2024.subsystems.vision;

import java.math.BigDecimal;
import java.text.DecimalFormat;
import java.util.Optional;

import com.spartronics4915.frc2024.LimelightHelpers;
import com.spartronics4915.frc2024.Constants.Vision.VisionPipelines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightDevice extends SubsystemBase {

    public static record VisionMeasurement(Pose3d pose, double timestamp) {}

    private final String mName;
    private final boolean mValid;
    private final Field2d mField;
    private VisionPipelines mPipeline;

    /**
     * Creates a new LimelightDevice. The pipeline is initialized to 0, which tracks April Tags.
     * @param name The name of the limelight
     */
    public LimelightDevice(String name) {
        String formattedName = "limelight-" + name;
        mName = formattedName;
        mValid = !NetworkTableInstance.getDefault().getTable(formattedName).getKeys().isEmpty();
        mField = new Field2d();
        mPipeline = VisionPipelines.FIDUCIALS_3D;
        // mPipeline = ((name.equals("alice")) ? VisionPipelines.ALICE_TEMP_NOTE_DETECTOR : VisionPipelines.FIDUCIALS_3D);
        LimelightHelpers.setPipelineIndex(mName, mPipeline.pipeline);
        createShuffleboard();
    }

    public Optional<VisionMeasurement> getVisionMeasurement() {
        if (!getTv() || mPipeline.isDetector) {
            return Optional.empty();
        }
        double[] botpose = LimelightHelpers.getBotPose_wpiBlue(mName);
        Pose3d pose = new Pose3d(botpose[0], botpose[1], botpose[2], new Rotation3d(botpose[3], botpose[4], botpose[5]));
        double timestamp = Timer.getFPGATimestamp() - (botpose[6]/1000.0);
        return Optional.of(new VisionMeasurement(pose, timestamp));
    }

    public double getTx() {
        return LimelightHelpers.getTX(mName);
    }
    
    public double getTy() {
        return LimelightHelpers.getTY(mName);
    }

    /**
     * @return If the limelight can see any tags
     */
    public boolean getTv() {
        if (!mValid) return false;
        return LimelightHelpers.getTV(mName);
    }

    /**
     * @return The {@link Pose3d} of the robot, as estimated from Limelight MetaTag
     * @apiNote Returns <code>new Pose3d()</code> if invalid or on a detector pieline
     */
    public Pose3d getBotPose3d() {
        if (!mValid) return new Pose3d();
        return LimelightHelpers.getBotPose3d(mName);
    }

    /**
     * @return The {@link Pose2d} of the robot, as estimated from Limelight MetaTag
     * @apiNote Returns <code>new Pose2d()</code> if invalid or on a detector pieline
     */
    public Pose2d getBotPose2d() {
        if (!mValid) return new Pose2d();
        return LimelightHelpers.getBotPose2d(mName);
    }

    /**
     * @return The number of tags seen by the limelight
     * @apiNote Returns <code>0</code> if invalid or on a detector pieline
     */
    public int numberOfTagsSeen() {
        if (!mValid) return 0;
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults(mName);
        LimelightHelpers.LimelightTarget_Fiducial[] fiducials = llresults.targetingResults.targets_Fiducials;
        int tagCount = fiducials.length;
        return tagCount;
    }

    public VisionPipelines getVisionPipeline() {
        return mPipeline;
    }

    public void setVisionPipeline(VisionPipelines pipeline) {
        mPipeline = pipeline;
        System.out.println("pipeline: " + mPipeline.pipeline);
        if (!pipeline.isDetector) profilePipelineSwitching(true);
        LimelightHelpers.setPipelineIndex(mName, pipeline.pipeline);
    }

    /**
     * @return The id of the primary in-view tag
     * @apiNote Returns <code>0</code> if invalid or on a detector pipeline
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
     * @return The average distance to the visible tags
     * @apiNote Returns <code>0.0</code> if invalid or on a detector pipeline
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
     * @return The number of objects detected by the limelight
     * @apiNote Returns <code>0</code> if invalid or not on a detector pieline
     */
    public int numberOfObjectsDetected() {
        if (!mValid) return 0;
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults(mName);
        LimelightHelpers.LimelightTarget_Detector[] detected = llresults.targetingResults.targets_Detector;
        int detectedCount = detected.length;
        if (detectedCount > 0) profilePipelineSwitching(false);
        return detectedCount;
    }

    /**
     * @return The lowest detected target
     */
    public Optional<LimelightHelpers.LimelightTarget_Detector> getSelectedDetectorTarget() {
        if (!mValid) return Optional.empty();
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults(mName);
        LimelightHelpers.LimelightTarget_Detector[] detected = llresults.targetingResults.targets_Detector;
        if (detected.length == 0) return Optional.empty();
        LimelightHelpers.LimelightTarget_Detector selectedDetector = detected[0];
        for (LimelightHelpers.LimelightTarget_Detector d : detected) {
            if (d.ty < selectedDetector.ty) selectedDetector = d;
        }
        return Optional.of(selectedDetector);
    }

    public double getDetectedConfidence() {
        if (!mValid) return 0.0;
        Optional<LimelightHelpers.LimelightTarget_Detector> detected = getSelectedDetectorTarget();
        if (detected.isEmpty()) return 0.0;
        return detected.get().confidence;
    }

    public String getDetectedClass() {
        if (!mValid) return "";
        Optional<LimelightHelpers.LimelightTarget_Detector> detected = getSelectedDetectorTarget();
        if (detected.isEmpty()) return "";
        return detected.get().className;
    }

    /**
     * @return The name of the limelight
     */
    public String getName() {
        return mName;
    }

    public void updateFieldPose() {
        mField.setRobotPose(getBotPose2d());
    }

    public String getFormattedStringTxTy() {
        return (Math.floor(getTx() * 100) / 100) + ";" + (Math.floor(getTy() * 100) / 100);
        // BigDecimal bigTx = new BigDecimal(getTx()).setScale(2);
        // BigDecimal bigTy = new BigDecimal(getTy()).setScale(2);
        // return (bigTx + "°;" + bigTy + "°");
    }

    public double[] getXYZOfPrimaryTag() {
        if (!mValid || mPipeline.isDetector) return new double[3];
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

    private double timestamp = 0.0;
    private boolean timestampSet = false;
    private boolean intervalSet = true;
    private double interval = 0.0;
    private double timesSwitched = 0.0;
    private double totalInterval = 0.0;

    // FIDUCIAL > DETECTOR: 0.25s avg, 0.3s expected max
    // looks like it takes 0.15 to 0.4 seconds
    /* TODO
     * assign each limelight a static ip
     * rest api to set limelight names without web display
     * flash alice
     * finalize pipelines and backup
     * add ifCoral to constructor
     * measure pose offsets for talos and implement
     * calibrate limelights
     * tune detector pipeline
     * use low resolution on neural networks
     * figure out how to get that nice high resolution camera stream
     */

    private void profilePipelineSwitching(boolean end) {
        if (!end && !timestampSet) {
            timestamp = Timer.getFPGATimestamp();
            timestampSet = true;
            intervalSet = false;
        } else if (end && !intervalSet) {
            interval = (Timer.getFPGATimestamp() - timestamp);
            timestampSet = false;
            intervalSet = true;
            timesSwitched++;
            totalInterval += interval;
        }
    }

    public double getProfiledInterval() {
        return interval;
    }

    public double getAverageProfiledInterval() {
        return totalInterval / timesSwitched;
    }
    

    public String shuffleboardFormattedName(String title) {
        String name = mName.split("-")[1];
        return name + " " + title;
    }

    public void createShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("VisionTab");
        final int offset = (mName.equals("limelight-bob") ? 5 : 0);
        tab.add(shuffleboardFormattedName("field"),
            mField)
            .withSize(3, 2).withPosition(0 + offset, 0);
        tab.addBoolean(shuffleboardFormattedName("is valid"),
            () -> {return mValid;})
            .withPosition(3 + offset, 0);
        tab.addBoolean(shuffleboardFormattedName("tv"),
            () -> {return getTv();})
            .withPosition(4 + offset, 0);
        tab.addString(shuffleboardFormattedName("pipeline"),
            () -> {return mPipeline.toString();})
            .withPosition(3 + offset, 1);
        tab.addString(shuffleboardFormattedName("tx ty"),
            () -> {return getFormattedStringTxTy();})
            .withPosition(4 + offset, 1);
        tab.addBoolean(shuffleboardFormattedName("FIDUCIALS"),
            () -> {return !mPipeline.isDetector;})
            .withPosition(0 + offset, 2);
        tab.addInteger(shuffleboardFormattedName("tag count"),
            () -> {return numberOfTagsSeen();})
            .withPosition(1 + offset, 2);
        tab.addInteger(shuffleboardFormattedName("primary tag"),
            () -> {return getPrimaryTag();}) 
            .withPosition(2 + offset, 2);
        tab.addDouble(shuffleboardFormattedName("avg dist"),
            () -> {return getAverageDistanceToVisibleTags();})
            .withPosition(3 + offset, 2);
        tab.addBoolean(shuffleboardFormattedName("DETECTOR"),
            () -> {return mPipeline.isDetector;})
            .withPosition(0 + offset, 3);
        tab.addInteger(shuffleboardFormattedName("detected"),
            () -> {return numberOfObjectsDetected();})
            .withPosition(1 + offset, 3);
        tab.addDouble(shuffleboardFormattedName("confidence"),
            () -> {return getDetectedConfidence();})
            .withPosition(2 + offset, 3);
        tab.addString(shuffleboardFormattedName("class"),
            () -> {return getDetectedClass();})
            .withPosition(3 + offset, 3);
        tab.addDouble(shuffleboardFormattedName("switch interval"),
            () -> {return getProfiledInterval();})
            .withPosition(4 + offset, 2);
        tab.addDouble(shuffleboardFormattedName("avg interval"),
            () -> {return getAverageProfiledInterval();})
            .withPosition(4 + offset, 3);
        }
}
