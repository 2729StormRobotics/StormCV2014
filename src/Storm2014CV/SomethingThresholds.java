/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Storm2014CV;

import edu.wpi.first.smartdashboard.camera.WPICameraExtension;
import edu.wpi.first.smartdashboard.camera.WPILaptopCameraExtension;
import edu.wpi.first.smartdashboard.properties.BooleanProperty;
import edu.wpi.first.smartdashboard.properties.ColorProperty;
import edu.wpi.first.smartdashboard.properties.DoubleProperty;
import edu.wpi.first.smartdashboard.properties.IntegerProperty;
import edu.wpi.first.wpijavacv.WPIBinaryImage;
import edu.wpi.first.wpijavacv.WPIColor;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIContour;
import edu.wpi.first.wpijavacv.WPIGrayscaleImage;
import edu.wpi.first.wpijavacv.WPIImage;
import edu.wpi.first.wpijavacv.WPIPoint;
import edu.wpi.first.wpijavacv.WPIPolygon;
import java.awt.Color;
import java.util.ArrayList;

/**
 *
 * @author Tim
 */
public class SomethingThresholds extends WPILaptopCameraExtension{
    
    public IntegerProperty blueThreshold = new IntegerProperty(this, "blue", 50);
    public IntegerProperty blueInvThreshold = new IntegerProperty(this, "blue inv", 100);
    public IntegerProperty redThreshold = new IntegerProperty(this, "red", 50);
    public IntegerProperty redInvThreshold = new IntegerProperty(this, "red inv", 100);
    public IntegerProperty greenThreshold = new IntegerProperty(this, "green", 50);
    public IntegerProperty greenInvThreshold = new IntegerProperty(this, "green inv", 100);
    public ColorProperty colorProp = new ColorProperty(this, "contour color", Color.CYAN);
    public IntegerProperty width = new IntegerProperty(this, "contour width", 5);
    public DoubleProperty percent = new DoubleProperty(this, "percentage off", 10);
    public IntegerProperty vertices = new IntegerProperty(this, "Vertices check", 6);
    public IntegerProperty distance = new IntegerProperty(this, "Radius for circle", 7);
    public IntegerProperty range = new IntegerProperty(this, "Margin of error", 7);
    public BooleanProperty testOnOff = new BooleanProperty(this, "Threshold test", false);
   
    @Override
   public WPIImage processImage (WPIColorImage rawImage){
       WPIContour [] contours;
       WPIGrayscaleImage blue; WPIGrayscaleImage red; WPIGrayscaleImage green;
       blue = rawImage.getBlueChannel();
       green = rawImage.getGreenChannel();
       red = rawImage.getRedChannel();
       WPIBinaryImage blueAnd;
       WPIBinaryImage redAnd;
       WPIBinaryImage greenAnd;
       blueAnd = blue.getThreshold(blueThreshold.getValue() - 1).getAnd(blue.getThresholdInverted(blueInvThreshold.getValue()));
       redAnd = red.getThreshold(redThreshold.getValue() - 1).getAnd(red.getThresholdInverted(redInvThreshold.getValue()));
       greenAnd = green.getThreshold(greenThreshold.getValue() - 1).getAnd(green.getThresholdInverted(greenInvThreshold.getValue()));
       WPIBinaryImage andThresholds;
       andThresholds = redAnd.getAnd(greenAnd).getAnd(blueAnd);
       if(testOnOff.getValue()) {
           return andThresholds;
       }
       WPIColor wpiColorProp = new WPIColor(colorProp.getValue());
       contours = andThresholds.findContours();
       ArrayList<WPIPolygon> circleChecked = new ArrayList<WPIPolygon> ();
       for(int x=0;x<contours.length;x++){       
           WPIPolygon poly = contours[x].approxPolygon(percent.getValue());
           if(poly.getNumVertices() >= vertices.getValue()){
               circleChecked.add(poly);
           }
       }
       for(int x=0;x<circleChecked.size();x++){
           boolean notCircle=false;
           double xAverage = 0, yAverage = 0;
           for(int y=0;y<circleChecked.get(x).getPoints().length;y++){
               xAverage+=circleChecked.get(x).getPoints()[y].getX();
               yAverage+=circleChecked.get(x).getPoints()[y].getY();
           }
           xAverage /= circleChecked.get(x).getPoints().length;
           yAverage /= circleChecked.get(x).getPoints().length;
           for(int y=0;y<circleChecked.get(x).getPoints().length;y++){
               if(!(distanceForm(xAverage, yAverage, circleChecked.get(x).getPoints()[y].getX(), circleChecked.get(x).getPoints()[y].getY()) < (distance.getValue() + range.getValue()) &&
                       distanceForm(xAverage, yAverage, circleChecked.get(x).getPoints()[y].getX(), circleChecked.get(x).getPoints()[y].getY()) > (distance.getValue() - range.getValue()))){
                   notCircle = true;
               }
           }
           if(!notCircle){
               rawImage.drawPolygon(circleChecked.get(x), wpiColorProp, width.getValue());
           }
       }
       return rawImage;
    }
       public double distanceForm(double x1, double y1, double x2, double y2){
           return (Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2)));
       }
}
