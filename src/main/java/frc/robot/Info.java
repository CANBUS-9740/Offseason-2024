package frc.robot;

public class Info {
    private double distance;
    private double deltaAngleDegrees;

    public Info(double distance, double deltaAngleDegrees) {
        this.distance = distance;
        this.deltaAngleDegrees = deltaAngleDegrees;
    }


    public double getAngleDegrees() {
        return deltaAngleDegrees;
    }

    public void setDeltaAngleDegrees(double deltaAngleDegrees) {
        this.deltaAngleDegrees = deltaAngleDegrees;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }
}
