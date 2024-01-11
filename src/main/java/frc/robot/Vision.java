package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Class to handle Robot Vision
 */
public class Vision {

    private final PhotonCamera camera;
    private PhotonPipelineResult result;

    private int numTargets;
    private List<PhotonTrackedTarget> trackedTargets;

    /**
     * Creates Vision Object
     * @param cameraInstance PhotonVision Camera Instance Identifier Name as String
     */
    public Vision(String cameraInstance) {
        camera = new PhotonCamera(cameraInstance) ;
        result = new PhotonPipelineResult() ;

        numTargets = 0 ;
    }

    /**
     * Updates Robot Vision Results (Do not call often to reduce network strain)
     */
    public void updateVision() {
        result = camera.getLatestResult() ;

        if(result.hasTargets())
        {
            trackedTargets = result.getTargets() ;
            numTargets = trackedTargets.size() ;
        } 
        else 
        {
            trackedTargets = List.of() ;
            numTargets = 0 ;
        }
    }

    /**
     * Function to Return the Number of Targets Being Tracked
     * @return Number of Targets Being Tracked as Double
     */
    public double numTargets() {
        return numTargets ;
    }

    /**
     * Function to Return Targets Being Tracked
     * @return Targets Being Tracked as List of PhotonTrackedTarget Objects
     */
    public List<PhotonTrackedTarget> getTargets() {
        return trackedTargets ;
    }
    
    /**
     * Function to Check if any vision targets are in sight and being tracked
     * @return True if at minimum one target is visible and being tracked
     */
    public boolean hasTargets() {
        return result.hasTargets();
    }
}
