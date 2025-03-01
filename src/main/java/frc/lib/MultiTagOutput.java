package frc.lib;

import java.util.List;

import org.photonvision.proto.Photon;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class MultiTagOutput {
    private MultiTargetPNPResult multitag;
    private PhotonTrackedTarget bestTarget;
    private double timestamp;

    public MultiTagOutput(MultiTargetPNPResult multitag, double timestamp, PhotonTrackedTarget bestTarget) {
        this.multitag = multitag;
        this.timestamp = timestamp;
        this.bestTarget = bestTarget;
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
