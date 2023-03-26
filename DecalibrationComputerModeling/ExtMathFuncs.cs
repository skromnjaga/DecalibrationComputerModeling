using static System.Math;

using OpenCvSharp;

namespace DecalibrationComputerModeling;

public class ExtMathFuncs
{
    public static Mat Rotm2Euler(Mat r)
    {
        double sy = Sqrt(r.At<double>(0, 0) * r.At<double>(0, 0) + r.At<double>(1, 0) * r.At<double>(1, 0));
        bool singular = sy < 1e-6;
            
        Mat eulerVec = Mat.Zeros(MatType.CV_64F, new[] {1, 3});
        if (!singular)
        {
            eulerVec.At<double>(0, 0) = Atan2(r.At<double>(1, 0), r.At<double>(0, 0));
            eulerVec.At<double>(0, 1) = Atan2(-r.At<double>(2, 0), sy);
            eulerVec.At<double>(0, 2) = Atan2(r.At<double>(2, 1), r.At<double>(2, 2));
        }
        else
        {
            eulerVec.At<double>(0, 0) = 0;
            eulerVec.At<double>(0, 1) = Atan2(-r.At<double>(2, 0), sy);
            eulerVec.At<double>(0, 2) = Atan2(-r.At<double>(1, 2), r.At<double>(1, 1));
        }
            
        return eulerVec;
    }

    public static Mat Euler2Rotm(Mat eulerZYX)
    {
        Mat rx = new Mat(3, 3, MatType.CV_64F, new [,]
        {
            {1, 0, 0},
            {0, Cos(eulerZYX.At<double>(0, 2)), -Sin(eulerZYX.At<double>(0, 2))},
            {0, Sin(eulerZYX.At<double>(0, 2)), Cos(eulerZYX.At<double>(0, 2))},
        });
        Mat ry = new Mat(3, 3, MatType.CV_64F, new [,]
        {
            {Cos(eulerZYX.At<double>(0, 1)), 0, Sin(eulerZYX.At<double>(0, 1))},
            {0, 1, 0},
            {-Sin(eulerZYX.At<double>(0, 1)), 0, Cos(eulerZYX.At<double>(0, 1))},
        });
        Mat rz = new Mat(3, 3, MatType.CV_64F, new [,]
        {
            {Cos(eulerZYX.At<double>(0, 0)), -Sin(eulerZYX.At<double>(0, 0)), 0},
            {Sin(eulerZYX.At<double>(0, 0)), Cos(eulerZYX.At<double>(0, 0)), 0},
            {0, 0, 1},
        });
        
        Mat r = rz * ry * rx;

        // for (int i = 0; i < r.Rows; i++)
        // {
        //     for (int j = 0; j < r.Cols; j++)
        //     {
        //         r.At<double>(i, j) = Round(r.At<double>(i, j), 4, MidpointRounding.AwayFromZero);
        //     }
        // }
        
        return r;
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