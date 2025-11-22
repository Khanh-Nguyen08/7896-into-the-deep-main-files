package org.firstinspires.ftc.teamcode.badCode;



public class Chamber{
    public static int NULL_COLOR = -1;
    public static int GREEN_COLOR = 0;
    public static int PURPLE_COLOR = 1;
    
    private boolean occupied;
    private int color;
    private double intakePosition;
    private double outtakePosition;
    
    public Chamber(boolean occupancy, int color, double intake, double outtake){
        this.occupied = occupancy;
        this.color = color;
        this.intakePosition = intake;
        this.outtakePosition = outtake;
    }
    
    public void setOccupancy(boolean occupancy){
        this.occupied = occupancy;
    }
    public void setColor(int color){
        this.color = color;
    }
    
    public boolean isOccupied(){
        return occupied;
    }
    public int getColor(){
        return color;
    }
    public double getIntakePos(){
        return intakePosition;
    }
    public double getOuttakePos(){
        return outtakePosition;
    }
    public double getIntakeDistance(double position){
        return Math.abs(intakePosition - position);
    }
    public double getOuttakeDistance(double position){
        return Math.abs(outtakePosition - position);
    }
}


