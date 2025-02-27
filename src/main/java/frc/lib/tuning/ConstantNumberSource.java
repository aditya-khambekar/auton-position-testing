package frc.lib.tuning;

import java.util.function.Consumer;

public class ConstantNumberSource implements NumberSource {
    private final String name;
    private final double value;

    public ConstantNumberSource(String name, double value) {
        this.name = name;
        this.value = value;
    }

    @Override
    public String name() {
        return name;
    }

    @Override
    public double valueAsDouble() {
        return value;
    }

    @Override
    public void addListener(Consumer<Double> listener) {}
}
