package org.firstinspires.ftc.teamcode;

public class ArmTowerPosition {
    private Double armPosition;
    private Double towerPosition;

    public ArmTowerPosition() {
    }

    public ArmTowerPosition(Double armPosition, Double towerPosition) {
        this.armPosition = armPosition;
        this.towerPosition = towerPosition;
    }

    public Double getArmPosition() {
        return armPosition;
    }

    public void setArmPosition(Double armPosition) {
        this.armPosition = armPosition;
    }

    public Double getTowerPosition() {
        return towerPosition;
    }

    public void setTowerPosition(Double towerPosition) {
        this.towerPosition = towerPosition;
    }
}