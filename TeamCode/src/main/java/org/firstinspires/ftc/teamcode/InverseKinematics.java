package org.firstinspires.ftc.teamcode;

public class InverseKinematics {

    // lastPosition - Last position (x, y) of Gripper
    // newPosition - New position (x, y) of Gripper
    // lastTarget - Last position of Arm & Tower
    // newTarget - New position of Arm & Tower
    public void inverse(Position lastPosition, Position newPosition, ArmTowerPosition lastTarget, ArmTowerPosition newTarget) {
        if (lastPosition.getX() != newPosition.getX() || lastPosition.getY() != newPosition.getX()) {
            double prelimCompute = newPosition.getX() / 542.75588018;
            if (Math.abs(prelimCompute) > 1) {
                newPosition.setX(lastPosition.getX());
                newPosition.setY(lastPosition.getY());
                newTarget.setTowerPosition(lastTarget.getTowerPosition());
                newTarget.setArmPosition(lastTarget.getArmPosition());
            } else {
                double armP1 = Math.asin(prelimCompute);
                double armP2 = Math.PI - armP1;
                if (armP2 > Math.PI) {
                    armP2 -= 2 * Math.PI;
                }
                if (armP2 < -1 * Math.PI) {
                    armP2 += 2 * Math.PI;
                }
                double towerP1 = newPosition.getY() - Math.cos(armP1) * 542.75588018;
                double towerP2 = newPosition.getY() - Math.cos(armP2) * 542.75588018;
                boolean positionValid1 = axisTravelCheck(armP1, towerP1);
                boolean positionValid2 = axisTravelCheck(armP2, towerP2);

                if (positionValid1 && positionValid2) {
                    //best solution is chosen based on which will require minimum change of arm angle and the other will be made invalid
                    double armChangeP1 = Math.abs(armP1 - lastTarget.getArmPosition());
                    double armChangeP2 = Math.abs(armP2 - lastTarget.getArmPosition());
                    if (armChangeP1 <= armChangeP2) {
                        positionValid2 = false;
                    } else {
                        positionValid1 = false;
                    }
                }
                //if only one solution will work need to set targets to that solution
                if (positionValid1) {
                    newTarget.setTowerPosition(towerP1);
                    newTarget.setArmPosition(armP1);
                } else {
                    if (positionValid2) {
                        newTarget.setTowerPosition(towerP2);
                        newTarget.setArmPosition(armP2);
                    } else {
                        newPosition.setX(lastPosition.getX());
                        newPosition.setY(lastPosition.getY());
                        newTarget.setTowerPosition(lastTarget.getTowerPosition());
                        newTarget.setArmPosition(lastTarget.getArmPosition());
                    }
                }
            }
        }
    }

    private boolean axisTravelCheck(double armAngle, double towerPosition) {
        if (towerPosition < 0 || towerPosition > 630 || armAngle < -2.889 || armAngle > 2.889) {
            return false;
        } else {
            return true;
        }
    }
}
