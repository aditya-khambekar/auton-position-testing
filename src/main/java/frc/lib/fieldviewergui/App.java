package frc.lib.fieldviewergui;

import frc.robot.constants.FieldConstants;

import javax.swing.*;
import java.awt.*;
import java.util.Objects;

public class App extends JFrame {
    public static void main(String[] args) {
        App.getInstance();
    }

    private static volatile App instance;

    public static synchronized App getInstance() {
        return Objects.requireNonNullElseGet(instance, () -> instance = new App());
    }

    private App(){
        super();
        setTitle("Reefscape Field Viewer");
        setSize(957, 484);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        add(new Field(), BorderLayout.CENTER);
        setVisible(true);
    }

    void update(){
        var perimeter = getWidth() + getHeight();
        var widthRatio = (957.0/(957 + 484));
        var heightRatio = (484.0/(957 + 484));
        setSize((int) (perimeter * widthRatio), (int) (perimeter * heightRatio));
    }
}
