package org.firstinspires.ftc.teamcode.brainstem.auto.poses;

// blue close field coords — edit these
// headings: 0°=+Y (into field), increases CCW; blue collect faces −Y = 180°
public class BlueClosePoses extends RobotPoses {
    public BlueClosePoses() {
        start = xyz(-65, -41.75, -90);
        lookAtOb = xyz(-23, -23, 75);
        openGatePos = xyz(-7, -72 + 6 + 5.25, 45);
        close1Shooting = xyz(-39, -39, 133);
        collect1Pre = xyz(-12, -31, 180);
        collect1Mid = xyz(-12, -22, 180);
        firstSpikeEnd = xyz(-12, -58, 180);
        strafePos = xyz(-17, -36, 180);
        collect2Mid = xyz(12, -25, 180);
        collect2Pre = xyz(12, -31, 180);
        secondSpikeEnd = xyz(12, -64, 180);
        collect3Pre = xyz(36, -31, 180);
        collect3PrePass = xyz(12, -45, 180);
        thirdSpikeEnd = xyz(36, -64, 180);
    }
}
