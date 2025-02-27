package frc.lib.tuning;

public class ConstantTableSource implements TableSource {
    protected final String name;
    protected final String[] columnNames;
    protected final double[][] data;
    private final int numRows;
    private final int numCols;

    public ConstantTableSource(String name, int numRows, int numCols, String[] columnNames, double[][] data) {
        this.name = name;
        this.numRows = numRows;
        this.numCols = numCols;
        this.columnNames = columnNames;
        this.data = data;
    }

    @Override
    public int numRows() {
        return numRows;
    }

    @Override
    public int numColumns() {
        return numCols;
    }

    @Override
    public String getColumnName(int col) {
        return columnNames[col];
    }

    @Override
    public double getCellAsDouble(int row, int col) {
        return data[row][col];
    }

    @Override
    public String name() {
        return name;
    }

    @Override
    public void addListener(TableSourceListener listener) {}
}
