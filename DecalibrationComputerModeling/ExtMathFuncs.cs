using static System.Math;

using OpenCvSharp;

namespace DecalibrationComputerModeling;


public class ExtMathFuncs
{
    /// <summary>
    /// Преобразовывает из матрицы поворота в углы Эйлера в формате Mat
    /// </summary>
    /// <param name="rotationMatrix">Матрица поворота</param>
    /// <returns>Углы Эйлера</returns>
    public static Mat RotationMatrixToEulerAnglesMat(Mat rotationMatrix)
    {
        double sy = Sqrt(rotationMatrix.At<double>(0, 0) * rotationMatrix.At<double>(0, 0) + rotationMatrix.At<double>(1, 0) * rotationMatrix.At<double>(1, 0));
        bool singular = sy < 1e-6;
            
        Mat eulerVec = Mat.Zeros(MatType.CV_64F, new[] {1, 3});
        if (!singular)
        {
            eulerVec.At<double>(0, 0) = Atan2(rotationMatrix.At<double>(1, 0), rotationMatrix.At<double>(0, 0));
            eulerVec.At<double>(0, 1) = Atan2(-rotationMatrix.At<double>(2, 0), sy);
            eulerVec.At<double>(0, 2) = Atan2(rotationMatrix.At<double>(2, 1), rotationMatrix.At<double>(2, 2));
        }
        else
        {
            eulerVec.At<double>(0, 0) = 0;
            eulerVec.At<double>(0, 1) = Atan2(-rotationMatrix.At<double>(2, 0), sy);
            eulerVec.At<double>(0, 2) = Atan2(-rotationMatrix.At<double>(1, 2), rotationMatrix.At<double>(1, 1));
        }
            
        return eulerVec;
    }

    /// <summary>
    /// Преобразовывает из матрицы поворота в углы Эйлера
    /// </summary>
    /// <param name="rotationMatrix">Матрица поворота</param>
    /// <returns>Углы Эйлера</returns>
    public static double[] RotationMatrixToEulerAngles(Mat rotationMatrix)
    {
        rotationMatrix.GetRectangularArray(out double[,] rotMatrix);

        return RotationMatrixToEulerAngles(rotMatrix);
    }

    /// <summary>
    /// Преобразовывает из матрицы поворота в углы Эйлера
    /// </summary>
    /// <param name="rotationMatrix">Матрица поворота</param>
    /// <returns>Углы Эйлера</returns>
    public static double[] RotationMatrixToEulerAngles(double[,] rotationMatrix)
    {
        double sy = Sqrt(rotationMatrix[0, 0] * rotationMatrix[0, 0] + rotationMatrix[1, 0] * rotationMatrix[1, 0]);
        bool singular = sy < 1e-6;

        double[] eulerVec = new double[3];
        if (!singular)
        {
            eulerVec[0] = Atan2(rotationMatrix[1, 0], rotationMatrix[0, 0]);
            eulerVec[1] = Atan2(-rotationMatrix[2, 0], sy);
            eulerVec[2] = Atan2(rotationMatrix[2, 1], rotationMatrix[2, 2]);
        }
        else
        {
            eulerVec[0] = 0;
            eulerVec[1] = Atan2(-rotationMatrix[2, 0], sy);
            eulerVec[2] = Atan2(-rotationMatrix[1, 2], rotationMatrix[1, 1]);
        }

        return eulerVec;
    }

    /// <summary>
    /// Преобразовывает из углов Эйлера в матрицу поворота в формате Mat
    /// </summary>
    /// <param name="eulerAnglesZYX">Углы Эйлера</param>
    /// <returns>Матрица поворота</returns>
    public static Mat EulerAnglesToRotationMatrixMat(Mat eulerAnglesZYX)
    {
        Mat rx = new Mat(3, 3, MatType.CV_64F, new [,]
        {
            {1, 0, 0},
            {0, Cos(eulerAnglesZYX.At<double>(0, 2)), -Sin(eulerAnglesZYX.At<double>(0, 2))},
            {0, Sin(eulerAnglesZYX.At<double>(0, 2)), Cos(eulerAnglesZYX.At<double>(0, 2))},
        });
        Mat ry = new Mat(3, 3, MatType.CV_64F, new [,]
        {
            {Cos(eulerAnglesZYX.At<double>(0, 1)), 0, Sin(eulerAnglesZYX.At<double>(0, 1))},
            {0, 1, 0},
            {-Sin(eulerAnglesZYX.At<double>(0, 1)), 0, Cos(eulerAnglesZYX.At<double>(0, 1))},
        });
        Mat rz = new Mat(3, 3, MatType.CV_64F, new [,]
        {
            {Cos(eulerAnglesZYX.At<double>(0, 0)), -Sin(eulerAnglesZYX.At<double>(0, 0)), 0},
            {Sin(eulerAnglesZYX.At<double>(0, 0)), Cos(eulerAnglesZYX.At<double>(0, 0)), 0},
            {0, 0, 1},
        });
        
        Mat r = rz * ry * rx;
        
        return r;
    }

    /// <summary>
    /// Преобразовывает из углов Эйлера в матрицу поворота в формате Mat
    /// </summary>
    /// <param name="eulerAnglesZYX">Углы Эйлера</param>
    /// <returns>Матрица поворота</returns>
    public static Mat EulerAnglesToRotationMatrixMat(double[] eulerAnglesZYX)
    {
        Mat rotationMatrixMatX = new Mat(3, 3, MatType.CV_64F, new[,]
        {
            {1, 0, 0},
            {0, Cos(eulerAnglesZYX[2]), -Sin(eulerAnglesZYX[2])},
            {0, Sin(eulerAnglesZYX[2]), Cos(eulerAnglesZYX[2])},
        });
        Mat rotationMatrixMatY = new Mat(3, 3, MatType.CV_64F, new[,]
        {
            {Cos(eulerAnglesZYX[1]), 0, Sin(eulerAnglesZYX[1])},
            {0, 1, 0},
            {-Sin(eulerAnglesZYX[1]), 0, Cos(eulerAnglesZYX[1])},
        });
        Mat rotationMatrixMatZ = new Mat(3, 3, MatType.CV_64F, new[,]
        {
            {Cos(eulerAnglesZYX[0]), -Sin(eulerAnglesZYX[0]), 0},
            {Sin(eulerAnglesZYX[0]), Cos(eulerAnglesZYX[0]), 0},
            {0, 0, 1},
        });

        Mat rotationMatrixMat = rotationMatrixMatZ * rotationMatrixMatY * rotationMatrixMatX;

        return rotationMatrixMat;
    }

    /// <summary>
    /// Преобразовывает из углов Эйлера в матрицу поворота
    /// </summary>
    /// <param name="eulerAnglesZYX">Углы Эйлера</param>
    /// <returns>Матрица поворота</returns>
    public static double[,] EulerAnglesToRotationMatrix(double[] eulerAnglesZYX)
    {
        Mat rotationMatrixMat = EulerAnglesToRotationMatrixMat(eulerAnglesZYX);
        rotationMatrixMat.GetRectangularArray(out double[,] rotationMatrix);

        return rotationMatrix;
    }

    /// <summary>
    /// Расчитывает ошибку репроекции для двух наборов двухмерных координат точек 
    /// по методу штрафных функций с учетом изменения в оптимизируемых параметрах
    /// </summary>
    /// <param name="coord2Dl1">Первый массив двухмерных координат для первой камеры</param>
    /// <param name="coord2Dr1">Первый массив двухмерных координат для второй камеры</param>
    /// <param name="coord2Dl2">Второй массив двухмерных координат для первой камеры</param>
    /// <param name="coord2Dr2">Второй массив двухмерных координат для второй камеры</param>
    /// <param name="oldVec">Исходный набор параметров</param>
    /// <param name="newVec">Оптимизированный набор параметров</param>
    /// <returns>Среднее значение ошибки репроекции с учетом штрафной функции</returns>
    public static double ReprojectionErrorMod(Mat coord2Dl1, Mat coord2Dr1, Mat coord2Dl2, Mat coord2Dr2,
        double[] oldVec, double[] newVec)
    {
        double err = Sqrt(((Mat)(((Mat) (coord2Dl1.Col(0) - coord2Dl2.Col(0))).Pow(2) 
                           + ((Mat) (coord2Dr1.Col(1) - coord2Dr2.Col(1))).Pow(2))).Sum()[0] / coord2Dl1.Rows);

        double tMax = 10.0;
        double rMax = 2 * PI / 180;

        for (int i = 0; i < 6; i++)
        {
            if (Abs(oldVec[i] - newVec[i]) > (i < 3 ? tMax : rMax))
            {
                err += 10000;
            }
        }

        return err;
    }
}