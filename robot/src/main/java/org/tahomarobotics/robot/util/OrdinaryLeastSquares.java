package org.tahomarobotics.robot.util;

import edu.wpi.first.math.Matrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.simple.SimpleMatrix;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

public class OrdinaryLeastSquares {

    private static final Logger logger = LoggerFactory.getLogger(OrdinaryLeastSquares.class);

    private final List<double[]> data = new ArrayList<>();
    private final int columns;

    public OrdinaryLeastSquares(int columns) {
        this.columns = columns;
    }

    public void add(double data[]) {
        if (data.length != columns + 1) {
            throw new RuntimeException("Incorrect number of data points");
        }
        this.data.add(data);
    }

    public double[] calculate() {

        int samples = data.size();


        double rawY[] = new double[samples];
        double rawX[] = new double[samples * columns];

        int index = 0;
        for (double raw[] : data) {
            rawY[index] = raw[0];
            for(int i = 0; i < columns; i++) {
                rawX[index+i] = raw[i+1];
            }
            index++;
        }

        Matrix Y = new Matrix(new SimpleMatrix(samples, 1, true, rawY));
        Matrix X = new Matrix(new SimpleMatrix(samples, columns, true, rawX));
        Matrix b = new Matrix(new SimpleMatrix(1, columns));

        LinearSolver<DMatrixRMaj, DMatrixRMaj> solver = LinearSolverFactory_DDRM.qr(X.getNumRows(),X.getNumCols());
        if( !solver.setA(X.getStorage().getDDRM()) ) {
            throw new RuntimeException("Singular matrix");
        }

        if( solver.quality() <= 1e-8 )
            throw new RuntimeException("Nearly singular matrix");


        solver.solve(Y.getStorage().getDDRM(),b.getStorage().getDDRM());

        //logger.info(String.format("%n%s", Y));

        //logger.info(String.format("%n%s", X));

        //logger.info(String.format("%n%s", b));

        double c[] = new double[columns];

        for (int i = 0; i < columns; i++) {
            c[i] = b.get(i, 0);
        }

        return c;
    }

}
