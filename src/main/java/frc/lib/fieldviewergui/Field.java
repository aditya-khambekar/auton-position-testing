package frc.lib.fieldviewergui;

import javax.imageio.ImageIO;
import javax.swing.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FieldConstants;

import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionListener;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.HashSet;

public class Field extends JPanel implements MouseMotionListener {
    private final double imperialWidth = 317; //26 ft 5 inches wide
    private final double imperialLength = 690.875;// 57 fft 6 7/8in wide
    private final double inchesToMeters = 0.0254;
    //314 x689

    BufferedImage fieldImage = getFieldImage();
    //double mousex, mousey = 0;
    int mousex, mousey = 0;
    double fieldMinimumX = 0.13377321603 * getWidth();
    double fieldMaximumX = (1 - 0.13377321603) * getWidth();
    double fieldMinimumY = 0.14634888438 * getHeight();
    double fieldMaximumY = (1 - 0.14634888438) * getHeight();

    HashSet<Pose2d> poses = new HashSet<>();

    private static BufferedImage getFieldImage() {
        try {
            return ImageIO.read(new File("src\\main\\java\\frc\\lib\\fieldviewergui\\2025 REEFSCAPE Transparent Background.png"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public Field(){
        super();
        addMouseMotionListener(this);
        setPreferredSize(new Dimension(943, 448));

        poses.add(FieldConstants.AutonStartingPositions.LEFT_EDGE.Pose);
        poses.add(FieldConstants.AutonStartingPositions.RIGHT_EDGE.Pose);
    }

    @Override
    public void paint(Graphics g){
        super.paint(g);
        this.update();
        g.setColor(Color.DARK_GRAY);
        g.fillRect(0, 0, getWidth(), getHeight());
        g.setColor(Color.GRAY);
        g.fillRect((int) fieldMinimumX, (int) fieldMinimumY, (int) (fieldMaximumX-fieldMinimumX), (int) (fieldMaximumY-fieldMinimumY));
        g.drawImage(fieldImage, 0, 0, getWidth(), getHeight(), this);
        g.setColor(Color.BLACK);
        g.drawRect((int) fieldMinimumX, (int) fieldMinimumY, (int) (fieldMaximumX-fieldMinimumX), (int) (fieldMaximumY-fieldMinimumY));
        g.setColor(new Color(255, 215, 0));
        if (fieldMinimumX < mousex
            && mousex < fieldMaximumX
        && fieldMinimumY < mousey
            && mousey < fieldMaximumY
        ) {
            g.drawLine(mousex, 0, mousex, getHeight());
            g.drawLine(0, mousey, getWidth(), mousey);
            String s = "(" +
                    getXValue() +
                    ", " +
                    getYValue() +
                    ")";
            g.drawString(s, mousex + 10, mousey - 10);
        }
        //maybe this will work
        g.drawRect(mousex-15,mousey-15,30,30);

        g.setColor(Color.RED);
        for(Pose2d pose:poses){
            for(Point[] side: generateRotatedSquare(pose.getX(), pose.getY(), 30, pose.getRotation().getDegrees())){
                var p0 = toGuiCoordinate(MetersToInches(side[0]));
                var p1 = toGuiCoordinate(MetersToInches(side[1]));
                System.out.println(p0.toString() + " " + p1.toString());
                //g.drawLine(p0.x, p0.y, p1.x, p1.y);
            }
            Point p = MetersToInches(new Point((int) pose.getX(), (int) pose.getY()));
            g.drawString("pose", toGuiCoordinate(p).x, toGuiCoordinate(p).y);
        }
    }

    public static Point[][] generateRotatedSquare(double centerX, double centerY, double sideLength, double rotationAngle) {
        // Convert rotation angle to radians
        double angleRadians = Math.toRadians(rotationAngle);

        // Calculate coordinates of the four corners of the square
        double[] xCoords = new double[4];
        double[] yCoords = new double[4];

        // Initial coordinates of the square (assuming no rotation)
        xCoords[0] = centerX - sideLength / 2; 
        yCoords[0] = centerY - sideLength / 2;
        xCoords[1] = centerX + sideLength / 2;
        yCoords[1] = centerY - sideLength / 2;
        xCoords[2] = centerX + sideLength / 2;
        yCoords[2] = centerY + sideLength / 2;
        xCoords[3] = centerX - sideLength / 2;
        yCoords[3] = centerY + sideLength / 2;

        // Rotate the coordinates
        for (int i = 0; i < 4; i++) {
            double x = xCoords[i] - centerX;
            double y = yCoords[i] - centerY;
            xCoords[i] = centerX + x * Math.cos(angleRadians) - y * Math.sin(angleRadians);
            yCoords[i] = centerY + x * Math.sin(angleRadians) + y * Math.cos(angleRadians);
        }

        // Create the array of points representing the sides
        Point[][] sides = new Point[4][2];
        sides[0][0] = new Point((int)xCoords[0], (int) yCoords[0]);
        sides[0][1] = new Point((int)xCoords[1], (int) yCoords[1]);
        sides[1][0] = new Point((int)xCoords[1], (int) yCoords[1]);
        sides[1][1] = new Point((int)xCoords[2], (int) yCoords[2]);
        sides[2][0] = new Point((int)xCoords[2], (int) yCoords[2]);
        sides[2][1] = new Point((int)xCoords[3], (int) yCoords[3]);
        sides[3][0] = new Point((int)xCoords[3], (int) yCoords[3]);
        sides[3][1] = new Point((int)xCoords[0], (int) yCoords[0]);

        return sides;
    }

    private double getYValue() {
        return toFieldCoordinate(new Point(0, mousey)).y;
    }

    private double getXValue() {
        return toFieldCoordinate(new Point(mousex, 0)).x;
    }

    @Override
    public void mouseDragged(MouseEvent e) {

    }

    @Override
    public void mouseMoved(MouseEvent e) {
        mousex = e.getX();
        mousey = e.getY();
        repaint();
    }

    void update(){
        fieldMinimumX = 0.13377321603 * getWidth();
        fieldMaximumX = (1 - 0.13377321603) * getWidth();
        fieldMinimumY = 0.14634888438 * getHeight();
        fieldMaximumY = (1 - 0.14634888438) * getHeight();
        App.getInstance().update();
    }

    Point toFieldCoordinate(Point guiCoordinate){
        var x = guiCoordinate.x;
        var y = guiCoordinate.y;

        x = (int) ((x - fieldMinimumX) * 957.065/App.getInstance().getWidth());
        y = (int) ((y - fieldMinimumY) * 484.34/App.getInstance().getHeight());
        y = 317 - y;
        return new Point(x, y);
    }

    Point toGuiCoordinate(Point fieldCoordinate) {
        var x = fieldCoordinate.x;
        var y = fieldCoordinate.y;

        y = 317 - y;

        x = (int) (((x / 957.065) * App.getInstance().getWidth()) + fieldMinimumX);
        y = (int) (((y / 484.34) * App.getInstance().getHeight()) + fieldMinimumY);

        return new Point(x, y);
    }

    Point MetersToInches(Point p){
        var x = Units.metersToInches(p.x);
        var y = Units.metersToInches(p.y);
        return new Point((int)x, (int)y);
    }

    Point InchesToMeters(Point p){
        var x = Units.inchesToMeters(p.x);
        var y = Units.inchesToMeters(p.y);
        return new Point((int)x, (int)y);
    }
}

