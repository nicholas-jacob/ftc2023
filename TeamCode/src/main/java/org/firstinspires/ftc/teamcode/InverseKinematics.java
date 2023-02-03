package org.firstinspires.ftc.teamcode;

public class InverseKinematics {

    private double armLength=542.75588018;
    private double TPR=0;
    private double TPMM=0;
    public int armTarget=0;
    public int towerTarget=0;

    public InverseKinematics (double ticksPerRadian, double ticksPerMM){
        TPR=ticksPerRadian;
        TPMM=ticksPerMM;
    }


    public boolean calculate (double targetX, double targetY, int armPos, int twPos){
        //quickly check is position is possible
        double prelimCompute = targetX/armLength;
        if (Math.abs(prelimCompute)>1) {
            return false;
        }
        double armP1=Math.asin(prelimCompute);
        double armP2=Math.PI-armP1;
        if (armP2 > Math.PI) {
            armP2 -= 2 * Math.PI;
        }
        if (armP2 < -1 * Math.PI) {
            armP2 += 2 * Math.PI;
        }
        double towerP1 = targetY - Math.cos(armP1) * 542.75588018;
        double towerP2 = targetY - Math.cos(armP2) * 542.75588018;

        boolean positionValid1=axisTravelCheck(armP1, towerP1);
        boolean positionValid2=axisTravelCheck(armP2, towerP2);

        if (positionValid2 && positionValid1){
            double armChange1 = Math.abs(armP1*TPR-armPos);
            double armChange2 = Math.abs(armP2*TPR-armPos);

            if (armChange1<armChange2){
                armTarget=(int)Math.round(armP1*TPR);
                towerTarget=(int)Math.round(towerP1*TPMM);
                return true;
            }
            else {
                armTarget=(int)Math.round(armP2*TPR);
                towerTarget=(int)Math.round(towerP2*TPMM);
                return true;
            }
        }
        else if(positionValid1){
            armTarget=(int)Math.round(armP1*TPR);
            towerTarget=(int)Math.round(towerP1*TPMM);
            return true;
        }
        else if(positionValid2){
            armTarget=(int)Math.round(armP2*TPR);
            towerTarget=(int)Math.round(towerP2*TPMM);
            return true;
        }
        else {
            return false;
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
