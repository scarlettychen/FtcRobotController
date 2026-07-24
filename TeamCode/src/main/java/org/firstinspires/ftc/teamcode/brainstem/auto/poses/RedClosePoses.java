package org.firstinspires.ftc.teamcode.brainstem.auto.poses;

// red close coords — tune separately from blue
// headings: 0°=+Y (into field), increases CCW
public class RedClosePoses extends RobotPoses {
    public RedClosePoses() {
        start = xyz(-65, 41.75, -90);
        lookAtOb = xyz(-22, 22, 105);
        openGatePos = xyz(-7, 72 - 6 - 5.25, 90);
        limelight = xyz(-24, 24, 135);
        close1Shooting = xyz(-38, 38, 45);
        collect1Pre = xyz(-13, 27, 0);
        collect1Mid = xyz(-13, 22, 0);
        firstSpikeEnd = xyz(-12, 51, 0);
        strafePos = xyz(-17, 36, 0);
        collect2Mid = xyz(9, 30, 0);
        collect2Pre = xyz(9, 24, 0);
        secondSpikeEnd = xyz(9, 51, 0);
        collect3Pre = xyz(0, 0, 0);
        collect3PrePass = xyz(0, 0, 0);
        thirdSpikeEnd = xyz(0, 0, 0);
    }
}
