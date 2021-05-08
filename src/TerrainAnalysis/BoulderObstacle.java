// Author: Ivan Ramos, <ivan-1081@hotmail.es>
// Open Source Software; you can modify and/or share it
package TerrainAnalysis;

import java.lang.String;
import java.text.DecimalFormat;

public class BoulderObstacle extends Obstacle{

    private final int x_cord = 0;
    private final int y_cord = 1;
    private final int z_cord = 2;

    private double[] leftMostPoint;
    private double[] rightMostPoint;
    private double[] fardestPoint;
    private double[] closestPoint;
    private double groundRef;
    private obstacleType type;
    private double[] centerCoords;
    private double height;
    
    public BoulderObstacle(double[] coords, double groundRefIn){
        leftMostPoint = coords.clone();
        rightMostPoint = coords.clone();
        fardestPoint = coords.clone();
        closestPoint = coords.clone();
        height = coords[z_cord];
        centerCoords = coords.clone();
        type = obstacleType.BOULDER;
        groundRef = groundRefIn;
    }

    public BoulderObstacle(int azimuth, int elevation, double distance, double groundRefIn){
        double[] coords = toCartesian(azimuth, elevation, distance);
        leftMostPoint = coords.clone();
        rightMostPoint = coords.clone();
        fardestPoint = coords.clone();
        closestPoint = coords.clone();
        height = coords[z_cord];
        centerCoords = coords.clone();
        groundRef = groundRefIn;
        type = obstacleType.BOULDER;
    }

    public void updateCenterCoords(){
        centerCoords[x_cord] = (rightMostPoint[x_cord] + leftMostPoint[x_cord]) / 2;
        centerCoords[y_cord] = (fardestPoint[y_cord] + closestPoint[y_cord]) / 2;
        centerCoords[z_cord] = (height - groundRef) / 2;
    }

    public double getArea(){
        double width = rightMostPoint[x_cord] - leftMostPoint[x_cord];
        return height * width;
    }

    public void updateBounds(double[] coords){

        boolean centerUpdateNeeded = false;
        if(leftMostPoint[x_cord] > coords[x_cord]){
            centerUpdateNeeded = true;
            leftMostPoint[x_cord] = coords[x_cord];
            leftMostPoint[y_cord] = coords[y_cord];
            leftMostPoint[z_cord] = coords[z_cord];
        }
        if(rightMostPoint[x_cord] < coords[x_cord]){
           centerUpdateNeeded = true;
            rightMostPoint[x_cord] = coords[x_cord];
            rightMostPoint[y_cord] = coords[y_cord];
            rightMostPoint[z_cord] = coords[z_cord];
        }
        if(fardestPoint[y_cord] < coords[y_cord]){
            centerUpdateNeeded = true;
            fardestPoint[x_cord] = coords[x_cord];
            fardestPoint[y_cord] = coords[y_cord];
            fardestPoint[z_cord] = coords[z_cord];
        }
        if(closestPoint[y_cord] > coords[y_cord]){
            centerUpdateNeeded = true;
            closestPoint[x_cord] = coords[x_cord];
            closestPoint[y_cord] = coords[y_cord];
            closestPoint[z_cord] = coords[z_cord];
        }
        if(this.height < coords[z_cord]){
            centerUpdateNeeded = true;
            height = coords[z_cord];
        }

        if(centerUpdateNeeded){
            updateCenterCoords();
        }

    }

    public void updateBounds(int azimuth, int elevation, double distance){
        //azimuth = 9000 - azimuth;
        //int az = (azimuth < 0) ? (36000 - (Math.abs(azimuth) % 36000) ) % 36000: (azimuth % 36000);
        double[] coords = toCartesian(azimuth, elevation, distance);
        updateBounds(coords);
    }

    public void combineObstacles(Obstacle o){

        updateBounds(o.getLeftMostPoint());
        updateBounds(o.getRightMostPoint());
        updateBounds(o.getFardestPoint());
        updateBounds(o.getClosestPoint());

    }

    public obstacleType getType(){
        return this.type;
    }
    
    public double[] getLeftMostPoint(){
        return leftMostPoint.clone();
    }
    
    public double[] getRightMostPoint(){
        return rightMostPoint.clone();
    }
    
    public double[] getFardestPoint(){
        return fardestPoint.clone();
    }
    
    public double[] getClosestPoint(){
        return closestPoint.clone();
    }
    
    public boolean isSameAs(Obstacle o){
        if(type != o.getType()){
            return false;
        }

        if(rightMostPoint[x_cord] > o.getLeftMostPoint()[x_cord] && leftMostPoint[x_cord] < o.getLeftMostPoint()[x_cord]){
            if(fardestPoint[y_cord] > o.getLeftMostPoint()[y_cord] && closestPoint[y_cord] < o.getLeftMostPoint()[y_cord]){
                return true;
            }
        }
        
        if(rightMostPoint[x_cord] > o.getRightMostPoint()[x_cord] && leftMostPoint[x_cord] < o.getRightMostPoint()[x_cord]){
            if(fardestPoint[y_cord] > o.getRightMostPoint()[y_cord] && closestPoint[y_cord] < o.getRightMostPoint()[y_cord]){
                return true;
            }
        }
        
        if(rightMostPoint[x_cord] > o.getFardestPoint()[x_cord] && leftMostPoint[x_cord] < o.getFardestPoint()[x_cord]){
            if(fardestPoint[y_cord] > o.getFardestPoint()[y_cord] && closestPoint[y_cord] < o.getFardestPoint()[y_cord]){
                return true;
            }
        }
        
        if(rightMostPoint[x_cord] > o.getClosestPoint()[x_cord] && leftMostPoint[x_cord] < o.getClosestPoint()[x_cord]){
            if(fardestPoint[y_cord] > o.getClosestPoint()[y_cord] && closestPoint[y_cord] < o.getClosestPoint()[y_cord]){
                return true;
            }
        }
        
        return false;
    }
    
    public String toString(){
      String s = "Boulder";
      
      DecimalFormat df = new DecimalFormat("#.##");
      
      String ret = "Obstacle Type: " + s + "\n";
      ret += "\tCentered at:\n";
      ret += "\t\tX: " + df.format(this.centerCoords[0]) + "\n";
      ret += "\t\tY: " + df.format(this.centerCoords[1]) + "\n";
      ret += "\t\tZ: " + df.format(this.centerCoords[2]) + "\n";
      ret += "\t\tHeight: " + df.format(this.height);
      return ret;
  }
}
