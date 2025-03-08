package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  // Cameras
  // Do not reference directly
  private PhotonCamera aprilTagCameraFront;
  private PhotonCamera aprilTagCameraRear;

  // Pipeline Results
  // These results are updated in the periodic
  // All methods should reference these results
  private PhotonPipelineResult aprilTagResultFront;
  private PhotonPipelineResult aprilTagResultRear;
  //private PhotonPipelineResult objectResult;
  private boolean aprilTagDetectedFront;
  private boolean aprilTagDetectedRear;

  // Pose Estimation
  private PhotonPoseEstimator visionEstimatorFront;
  private PhotonPoseEstimator visionEstimatorRear;
  private Matrix<N3, N1> curStdDevs;

  // Simulation
  private PhotonCameraSim aprilTagSimFront;
  private PhotonCameraSim aprilTagSimRear;
  private VisionSystemSim visionSim;


  /** Creates a new Vision. */
  public Vision() {
    aprilTagCameraFront = new PhotonCamera("April_Tag_Camera_Front");
    aprilTagCameraRear = new PhotonCamera("April_Tag_Camera_Rear");

    
    visionEstimatorFront = new PhotonPoseEstimator(Constants.kTagLayout , PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.kRobotToCamFront);
    visionEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    visionEstimatorRear = new PhotonPoseEstimator(Constants.kTagLayout , PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.kRobotToCamRear);
    visionEstimatorRear.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // ----- Simulation
    if (Robot.isSimulation()) {
        // Create the vision system simulation which handles cameras and targets on the field.
        visionSim = new VisionSystemSim("main");
        // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
        visionSim.addAprilTags(Constants.kTagLayout);
        // Create simulated camera properties. These can be set to mimic your actual camera.
        var cameraPropFront = new SimCameraProperties();
        cameraPropFront.setCalibration(480, 320, Rotation2d.fromDegrees(70));
        cameraPropFront.setCalibError(0.35, 0.10);
        cameraPropFront.setFPS(15);
        cameraPropFront.setAvgLatencyMs(50);
        cameraPropFront.setLatencyStdDevMs(15);

        //Might have to change later since this camera will be
        var cameraPropRear = new SimCameraProperties();
        cameraPropRear.setCalibration(480, 320, Rotation2d.fromDegrees(60)); // Adjusted from 70° to 60°
        cameraPropRear.setCalibError(0.35, 0.10);
        cameraPropRear.setFPS(15);
        cameraPropRear.setAvgLatencyMs(50);
        cameraPropRear.setLatencyStdDevMs(15);

        // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
        // targets.
        aprilTagSimFront = new PhotonCameraSim(aprilTagCameraFront, cameraPropFront);
        aprilTagSimRear = new PhotonCameraSim(aprilTagCameraRear, cameraPropRear);
        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(aprilTagSimFront, Constants.kRobotToCamFront);
        visionSim.addCamera(aprilTagSimRear, Constants.kRobotToCamRear);

        aprilTagSimFront.enableDrawWireframe(true);
        aprilTagSimRear.enableDrawWireframe(true);
    }
  }


      /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public List<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEstFront = Optional.empty();
        Optional<EstimatedRobotPose> visionEstRear = Optional.empty();
        List<EstimatedRobotPose> poses = new ArrayList<EstimatedRobotPose>();
        if (aprilTagDetectedFront) {
            visionEstFront = visionEstimatorFront.update(aprilTagResultFront);
            updateEstimationStdDevs(visionEstFront, aprilTagResultFront.getTargets());
            visionEstFront.ifPresent( est -> poses.add(est));

            // ----- Simulation
            if (Robot.isSimulation()) {
                visionEstFront.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject("VisionEstimationFront")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimationFront").setPoses();
                        });
            }
        }
            


        if (aprilTagDetectedRear) {
            visionEstRear = visionEstimatorRear.update(aprilTagResultRear);
            updateEstimationStdDevs(visionEstRear, aprilTagResultRear.getTargets());
            visionEstRear.ifPresent( est -> poses.add(est));

            // ----- Simulation
            if (Robot.isSimulation()) {
                visionEstRear.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject("VisionEstimationRear")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimationRear").setPoses();
                        });
            }
        }
        
        // If both cameras detected tags, prioritize the one with a lower estimated error
        if (visionEstFront.isPresent() && visionEstRear.isPresent()) {
            EstimatedRobotPose bestPose = visionEstFront.get().timestampSeconds > visionEstRear.get().timestampSeconds
                ? visionEstFront.get() : visionEstRear.get();
            return List.of(bestPose);
        }
        
        return poses;
    
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = visionEstimatorFront.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = Constants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Update April Tag Front Results
    aprilTagDetectedFront = false;
    var tempTagResults = aprilTagCameraFront.getAllUnreadResults();
    if (!tempTagResults.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      aprilTagResultFront = tempTagResults.get(tempTagResults.size() - 1);
      if (aprilTagResultFront.hasTargets()) {
          // At least one AprilTag was seen by the camera
          aprilTagDetectedFront = true;
      }
    }
    
    //Update April Tag Rear Results
    aprilTagDetectedRear = false;
    tempTagResults = aprilTagCameraRear.getAllUnreadResults();
    if (!tempTagResults.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      aprilTagResultRear = tempTagResults.get(tempTagResults.size() - 1);
      if (aprilTagResultRear.hasTargets()) {
          // At least one AprilTag was seen by the camera
          aprilTagDetectedRear = true;
      }
    }

    List<EstimatedRobotPose> pose = getEstimatedGlobalPose();
    if( !pose.isEmpty() ) {
        pose.forEach(
        est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs();

            // SubsystemsInst.getInst().drivetrain.addVisionMeasurement(
                    // est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });
    }
  }



  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
      visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
      if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
      if (!Robot.isSimulation()) return null;
      return visionSim.getDebugField();
  }
}
