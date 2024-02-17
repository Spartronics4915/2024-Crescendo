package com.spartronics4915.frc2024.subsystems.vision;

import java.util.Optional;

import com.spartronics4915.frc2024.LimelightHelpers;
import com.spartronics4915.frc2024.Constants.Vision.PoseOffsetConstants;
import com.spartronics4915.frc2024.Constants.Vision.VisionPipelines;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightDevice extends SubsystemBase {

    public static record VisionMeasurement(Pose3d pose, double timestamp) {}

    private final String mName;
    private boolean mValid = false;
    private VisionPipelines mPipeline;
    private final boolean mHasCoral;
    private final Field2d mField;
    private final Transform3d offset;
    private final Transform2d offset2d;
    private final SlewRateLimiter mRateLimiter;

    /**
     * Creates a new LimelightDevice. The pipeline is initialized to 0, which tracks April Tags.
     * @param name The name of the limelight
     */
    public LimelightDevice(String name, boolean hasCoral) {
        String formattedName = "limelight-" + name;
        mName = formattedName;
        if (name.equals("alice")) offset = PoseOffsetConstants.kAlicePoseOffset.inverse();
        else offset = PoseOffsetConstants.kBobPoseOffset.inverse();
        offset2d = new Transform2d(offset.getTranslation().toTranslation2d(), offset.getRotation().toRotation2d());
        mField = new Field2d();
        mPipeline = VisionPipelines.FIDUCIALS_3D;
        checkIfValid();
        mHasCoral = hasCoral;
        mRateLimiter = new SlewRateLimiter(246);
        createShuffleboard();
    }

    public void checkIfValid() {
        if (!NetworkTableInstance.getDefault().getTable(mName).getKeys().isEmpty()) {
            mValid = true;
            LimelightHelpers.setPipelineIndex(mName, mPipeline.pipeline);
            // LimelightHelpers.setLEDMode_ForceOff(mName);
        }
    }

    public boolean pipelineLoaded() {
        return (((int) LimelightHelpers.getCurrentPipelineIndex(mName)) == mPipeline.pipeline);
    }

    public Optional<VisionMeasurement> getVisionMeasurement() {
        if (!getTv() || mPipeline.isDetector || !pipelineLoaded()) {
            return Optional.empty();
        }
        // if (numberOfTagsSeen() < 2) return Optional.empty();
        double[] botpose = LimelightHelpers.getBotPose_wpiBlue(mName);
        Pose3d pose = new Pose3d(botpose[0], botpose[1], botpose[2], new Rotation3d(Units.degreesToRadians(botpose[3]), Units.degreesToRadians(botpose[4]), Units.degreesToRadians(botpose[5]))).transformBy(offset);
        double timestamp = Timer.getFPGATimestamp() - (botpose[6]/1000.0);
        return Optional.of(new VisionMeasurement(pose, timestamp));
    }

//#region Limelight Values
    public double getTx() {
        return LimelightHelpers.getTX(mName);
    }

    public double getTxLowpass() {
        return mRateLimiter.calculate(getTx());
    }
    
    public double getTy() {
        return LimelightHelpers.getTY(mName);
    }

    public boolean getTv() {
        if (!mValid) return false;
        return LimelightHelpers.getTV(mName);
    }

    /**
     * @return The {@link Pose3d} of the robot, as estimated from Limelight MetaTag
     * @apiNote Returns <code>new Pose3d()</code> if invalid
     */
    public Pose3d getBotPose3d() {
        if (!mValid) return new Pose3d();
        return LimelightHelpers.getBotPose3d(mName);
    }

    /**
     * @return The {@link Pose2d} of the robot, as estimated from Limelight MetaTag
     * @apiNote Returns <code>new Pose2d()</code> if invalid
     */
    public Pose2d getBotPose2d() {
        if (!mValid) return new Pose2d();
        return LimelightHelpers.getBotPose2d(mName);
    }

    private Pose2d getBotPose2d_wpiBlue() {
        if (!mValid) return new Pose2d();
        return LimelightHelpers.getBotPose2d_wpiBlue(mName);//.transformBy(offset2d);
    }
//#endregion

//#region Fiducials
    /**
     * @return The number of tags seen by the limelight
     * @apiNote Returns <code>0</code> if invalid
     */
    public int numberOfTagsSeen() {
        if (!mValid) return 0;
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults(mName);
        LimelightHelpers.LimelightTarget_Fiducial[] fiducials = llresults.targetingResults.targets_Fiducials;
        int tagCount = fiducials.length;
        return tagCount;
    }

    /**
     * @return The id of the primary in-view tag
     * @apiNote Returns <code>0</code> if invalid
     */
    public int getPrimaryTag() {
        if (!mValid) return 0;
        return (int) LimelightHelpers.getFiducialID(mName);

    }

    private double distanceToTag(LimelightHelpers.LimelightTarget_Fiducial tag) {
        Pose3d pose = tag.getTargetPose_CameraSpace(); //this should be the right method
        return Math.sqrt(pose.getX() * pose.getX() + pose.getY() * pose.getY() + pose.getZ() * pose.getZ());
    }

    /**
     * @return The average distance to the visible tags
     * @apiNote Returns <code>0.0</code> if invalid
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
//#endregion

//#region Detector
    /**
     * @return The number of objects detected by the limelight
     * @apiNote Returns <code>0</code> if invalid or not on a detector pieline
     */
    public int numberOfObjectsDetected() {
        if (!mValid) return 0;
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults(mName);
        LimelightHelpers.LimelightTarget_Detector[] detected = llresults.targetingResults.targets_Detector;
        int detectedCount = detected.length;
        return detectedCount;
    }

    /**
     * The selected detector target used by the limelight is determined by the sorting mode set on the pipeline.
     * If the sorting mode on the limelight is different than in this method, they can return different targets!
     * @return The lowest detected target
     */
    public Optional<LimelightHelpers.LimelightTarget_Detector> getSelectedDetectorTarget() {
        if (!mValid) return Optional.empty();
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults(mName);
        LimelightHelpers.LimelightTarget_Detector[] detected = llresults.targetingResults.targets_Detector;
        if (detected.length == 0) return Optional.empty();
        LimelightHelpers.LimelightTarget_Detector selectedDetector = detected[0];
        for (LimelightHelpers.LimelightTarget_Detector d : detected) { //LOWEST
            if (d.ty < selectedDetector.ty) selectedDetector = d;
        }
        /*
        for (LimelightHelpers.LimelightTarget_Detector d : detected) { //LARGEST
            if (d.ta < selectedDetector.ta) selectedDetector = d;
        }
        */
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
//#endregion

//#region Getters and Setters
    public String getName() {
        return mName;
    }

    public boolean hasCoral() {
        return mHasCoral;
    }

    public VisionPipelines getVisionPipeline() {
        return mPipeline;
    }

    public double getTruePipelineIndex() {
        return LimelightHelpers.getCurrentPipelineIndex(mName);
    }

    public boolean isValid() {
        return mValid;
    }

    /**
     * Switching pipelines takes 0.15 to 0.4 seconds
     */
    public void setVisionPipeline(VisionPipelines pipeline) {
        if (!mValid) return;
        if (intervalSet == false) System.out.println("Switching pipelines when interval is not set!");
        if (pipeline.isDetector && !mHasCoral) {
            System.out.println("WARNING! attempted to switch to pipeline " + mPipeline + " but there is no coral!");
            return;
        }
        mPipeline = pipeline;
        timestamp = Timer.getFPGATimestamp();
        intervalSet = false;
        LimelightHelpers.setPipelineIndex(mName, pipeline.pipeline);
    }
//#endregion

//#region Shuffleboard
    public void updateFieldPose() {
        mField.setRobotPose(getBotPose2d_wpiBlue());
    }

    public String getFormattedStringTxTy() {
        return (Math.floor(getTx() * 100) / 100) + ";" + (Math.floor(getTy() * 100) / 100);
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
    // private boolean timestampSet = false;
    private boolean intervalSet = true;
    private double interval = 0.0;
    private double timesSwitched = 0.0;
    private double totalInterval = 0.0;

    public void profilePipelineSwitching() {
        // if (!end && !timestampSet) {
        //     timestamp = Timer.getFPGATimestamp();
        //     timestampSet = true;
        //     intervalSet = false;
        // } else if (end && !intervalSet) {
        //     interval = (Timer.getFPGATimestamp() - timestamp);
        //     timestampSet = false;
        //     intervalSet = true;
        //     timesSwitched++;
        //     totalInterval += interval;
        // }
        if (!intervalSet && (LimelightHelpers.getCurrentPipelineIndex(mName) == mPipeline.pipeline))  {
            interval = (Timer.getFPGATimestamp() - timestamp);
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

    public double[] getFieldBotpose() {
        return new double[]{LimelightHelpers.getBotPose_wpiBlue(mName)[0], LimelightHelpers.getBotPose_wpiBlue(mName)[1], Units.degreesToRadians(LimelightHelpers.getBotPose_wpiBlue(mName)[5])};
    }

    public String shuffleboardFormattedName(String title) {
        String name = mName.split("-")[1];
        return name + " " + title;
    }

    public void createShuffleboard() {
        // ShuffleboardTab botposes = Shuffleboard.getTab("Botposes");
        // botposes.addDouble(mName + " botpose 0", () -> {return LimelightHelpers.getBotPose_wpiBlue(mName)[0];});
        // botposes.addDouble(mName + " botpose 1", () -> {return LimelightHelpers.getBotPose_wpiBlue(mName)[1];});
        // botposes.addDouble(mName + " botpose 2", () -> {return LimelightHelpers.getBotPose_wpiBlue(mName)[2];});
        // botposes.addDouble(mName + " botpose 3", () -> {return LimelightHelpers.getBotPose_wpiBlue(mName)[3];});
        // botposes.addDouble(mName + " botpose 4", () -> {return LimelightHelpers.getBotPose_wpiBlue(mName)[4];});
        // botposes.addDouble(mName + " botpose 5", () -> {return LimelightHelpers.getBotPose_wpiBlue(mName)[5];});
        // botposes.addDouble(mName + " botpose 6", () -> {return LimelightHelpers.getBotPose_wpiBlue(mName)[6];});
        // botposes.addDoubleArray(mName + " botpose pose", () -> {return getFieldBotpose();});

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
//#endregion
}
