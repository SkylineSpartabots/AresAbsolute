package frc.lib;

import org.photonvision.proto.Photon;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class MultiTagOutput extends MultiTargetPNPResult {
    private double timestamp;
    private PhotonTrackedTarget bestTarget;
    private MultiTargetPNPResult multitag;

    public MultiTagOutput(MultiTargetPNPResult multitag, double timestamp, PhotonTrackedTarget bestTarget) {
        this.timestamp = timestamp;
        this.bestTarget = bestTarget;
    }

    public MultiTagOutput(MultiTagOutput multiTagOutput, PhotonPipelineResult pipelineResult) {
        this.timestamp = pipelineResult.getTimestampSeconds();
        this.bestTarget = pipelineResult.getBestTarget();
    }

    public double getTimestamp() {
        return timestamp;
    }

    public PhotonTrackedTarget getBestTarget() {
        return bestTarget;
    }

    public MultiTargetPNPResult getMultiTag() {
        return multitag;
    }
}
