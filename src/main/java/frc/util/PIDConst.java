package frc.util;

public class PIDConst {
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kFF = 0.0;
    

    public PIDConst(double kP) {
        this.kP = kP;
    }

    public PIDConst(double kP, double kI) {
        this.kP = kP;
        this.kI = kI;
    }

    public PIDConst(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDConst(double kP, double kI, double kD, double kFF) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        this.kFF = kFF;
    } 


    public void setP(double kP) {
        this.kP = kP;
    }

    public void setI(double kI) {
        this.kI = kI;
    }

    public void setD(double kD) {
        this.kD = kD;
    }
    
    public void setFF(double kFF) {
        this.kFF = kFF;
    }



    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }
    
    public double getFF() {
        return kFF;
    }


    
}
