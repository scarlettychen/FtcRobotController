package org.firstinspires.ftc.teamcode.brainstem.auto.poses;

/** Team-editable Red close coordinates; mirror/tune independently of Blue. */
public class RedClosePoses extends RobotPoses {
    public RedClosePoses() {
        start = xyz(-65, 41.75, 0);
        lookAtOb = xyz(-22, 22, 195);
        openGatePos = xyz(-7, 72 - 6 - 5.25, -180);
        limelight = xyz(-24, 24, -135);
        close1Shooting = xyz(-38, 38, 135);
        collect1Pre = xyz(-13, 27, 90);
        collect1Mid = xyz(-13, 22, 90);
        firstSpikeEnd = xyz(-12, 51, 90);
        strafePos = xyz(-17, 36, 90);
        collect2Mid = xyz(9, 30, 90);
        collect2Pre = xyz(9, 24, 90);
        secondSpikeEnd = xyz(9, 51, 90);
        collect3Pre = xyz(0, 0, 90);
        collect3PrePass = xyz(0, 0, 90);
        thirdSpikeEnd = xyz(0, 0, 90);
    }
}
