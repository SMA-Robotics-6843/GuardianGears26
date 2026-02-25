/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants.*;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera1;
    private final PhotonCamera camera2;
    private final PhotonPoseEstimator photonEstimator;
    private PhotonPipelineResult result1;
    private PhotonPipelineResult result2;
    private int targetId;
    private List<PhotonTrackedTarget> targets;
    private List<Integer> excludedIds = List.of(4, 5, 14, 15, 12, 13, 1, 2, 3, 16);

    private Matrix<N3, N1> curStdDevs;

    public VisionSubsystem() {
        camera1 = new PhotonCamera(kCameraName);
        camera2 = new PhotonCamera(kCameraName2);

        photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        result1 = camera1.getLatestResult();
        result2 = camera1.getLatestResult();
        targets = result1.getTargets();

        if (!targets.isEmpty()) {
            targetId = 30; // Default value if no valid target is found
            for (PhotonTrackedTarget target : targets) {
                if (!excludedIds.contains(target.getFiducialId())) {
                    targetId = target.getFiducialId();
                    break; // Stop once a valid target is found
                }
            }
        } else {
            targetId = 40; // Default value if no targets are available
        }
        SmartDashboard.putNumber("targetId: ", targetId);
    }

    private static final Map<Integer, Pose2d> LEFT_ALIGNMENT_POSES = Map.ofEntries(
            // Blue alliance, front face, left side
            Map.entry(18, new Pose2d(3.15, 4.18, Rotation2d.fromDegrees(0))),
            // Blue alliance, front left face, left side
            Map.entry(19, new Pose2d(3.96, 5.25, Rotation2d.fromDegrees(-60))),
            // Blue alliance, back left face, left side
            Map.entry(20, new Pose2d(5.28, 5, Rotation2d.fromDegrees(-120))),
            // Blue alliance, back face, left side
            Map.entry(21, new Pose2d(5.82, 3.86, Rotation2d.fromDegrees(180))),
            // Blue alliance, back right face, left side
            Map.entry(22, new Pose2d(5, 2.79, Rotation2d.fromDegrees(120))),
            // Blue alliance, front right face, left side
            Map.entry(17, new Pose2d(3.68, 2.95, Rotation2d.fromDegrees(60))),
            // Red alliance, front face, left side
            Map.entry(7, new Pose2d(14.39, 3.87, Rotation2d.fromDegrees(180))),
            // Red alliance, front left face, left side
            Map.entry(6, new Pose2d(13.58, 2.8, Rotation2d.fromDegrees(120))),
            // Red alliance, back left face, left side
            Map.entry(11, new Pose2d(12.25, 3, Rotation2d.fromDegrees(60))),
            // Red alliance, back face, left side
            Map.entry(10, new Pose2d(11.73, 4.18, Rotation2d.fromDegrees(0))),
            // Red alliance, back right face, left side
            Map.entry(9, new Pose2d(12.54, 5.26, Rotation2d.fromDegrees(-60))),
            // Red alliance, front right face, left side
            Map.entry(8, new Pose2d(13.86, 5, Rotation2d.fromDegrees(-120))));


    private static final Map<Integer, Pose2d> RIGHT_ALIGNMENT_POSES = Map.ofEntries(
            // Blue alliance, front face, right side
            Map.entry(18, new Pose2d(3.15, 3.85, Rotation2d.fromDegrees(0))),
            // Blue alliance, front left face, right side
            Map.entry(19, new Pose2d(3.68, 5, Rotation2d.fromDegrees(-60))),
            // Blue alliance, back left face, right side
            Map.entry(20, new Pose2d(5, 5.25, Rotation2d.fromDegrees(-120))),
            // Blue alliance, back face, right side
            Map.entry(21, new Pose2d(5.82, 4.19, Rotation2d.fromDegrees(180))),
            // Blue alliance, back right face, right side
            Map.entry(22, new Pose2d(5.3, 2.96, Rotation2d.fromDegrees(120))),
            // Blue alliance, front right face, right side
            Map.entry(17, new Pose2d(3.965, 2.79, Rotation2d.fromDegrees(60))),
            // Red alliance, front face, right side
            Map.entry(7, new Pose2d(14.39, 4.19, Rotation2d.fromDegrees(180))),
            // Red alliance, front left face, right side
            Map.entry(6, new Pose2d(13.87, 2.96, Rotation2d.fromDegrees(120))),
            // Red alliance, back left face, right side
            Map.entry(11, new Pose2d(12.54, 2.79, Rotation2d.fromDegrees(60))),
            // Red alliance, back face, right side
            Map.entry(10, new Pose2d(11.73, 3.85, Rotation2d.fromDegrees(0))),
            // Red alliance, back right face, right side
            Map.entry(9, new Pose2d(12.25, 5, Rotation2d.fromDegrees(-60))),
            // Red alliance, front right face, right side
            Map.entry(8, new Pose2d(13.58, 5.26, Rotation2d.fromDegrees(-120))));
            

    public Pose2d decidePoseAlignmentLeft() {
        System.out.println("targetId left: " + targetId);
        return LEFT_ALIGNMENT_POSES.getOrDefault(targetId, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }

    public Pose2d decidePoseAlignmentRight() {
        System.out.println("targetId right: " + targetId);
        return RIGHT_ALIGNMENT_POSES.getOrDefault(targetId, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should
     * only be called once per loop.
     *
     * <p>
     * Also includes updates for the standard deviations, which can (optionally) be
     * retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets
     *         used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        List<PhotonPipelineResult> camera1UnreadResults = camera1.getAllUnreadResults();
        List<PhotonPipelineResult> camera2UnreadResults = camera2.getAllUnreadResults();
        // Combine camera1 results with camera2 results
        camera1UnreadResults.addAll(camera2UnreadResults);

        for (var change : camera1UnreadResults) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets()); 
        } 

        return visionEst;
    }
    

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from
     * the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets)
             {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
}
