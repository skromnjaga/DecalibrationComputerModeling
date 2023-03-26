using System;
using System.Linq;
using System.Collections.Generic;

using OpenCvSharp;


namespace DecalibrationComputerModeling
{
    public static class Helper
    {
        /// <summary>
        /// Рассчитывает триангуляцию двух наборов двухмерных точек для стереосистемы камер
        /// </summary>
        /// <param name="camera1">Первая камера стереосистемы</param>
        /// <param name="camera2">Вторая камера стереосистемы</param>
        /// <param name="camera1Points">Массив двухмерных координат для первой камеры</param>
        /// <param name="camera2Points">Массив двухмерных координат для второй камеры</param>
        /// <returns>Массив триангулированных трехмерных точек</returns>
        public static Point3d[] TriangulatePoints(CameraModel camera1, CameraModel camera2, Point2d[] camera1Points, Point2d[] camera2Points)
        {
            // применение триангуляции
            //Vec4d[] resCoord4D = Cv2.TriangulatePoints(camera1.ProjectionMat, camera2.ProjectionMat, camera1Points, camera2Points);
            Mat triangulatedPoints = new Mat();
            Cv2.TriangulatePoints(camera1.ProjectionMatrixMat, camera2.ProjectionMatrixMat, Mat.FromArray(camera1Points), Mat.FromArray(camera2Points), triangulatedPoints);

            // Избавляемся от вектора-столбца, который отвечает за масштабируемость
            Cv2.ConvertPointsFromHomogeneous(triangulatedPoints.T(), triangulatedPoints);

            triangulatedPoints.GetArray(out Point3d[] pointsCoords2D);

            return pointsCoords2D;
        }

        /// <summary>
        /// Рассчитывает триангуляцию двух наборов двухмерных точек для стереосистемы камер в формате Mat
        /// </summary>
        /// <param name="camera1">Первая камера стереосистемы</param>
        /// <param name="camera2">Вторая камера стереосистемы</param>
        /// <param name="camera1Points">Массив двухмерных координат для первой камеры</param>
        /// <param name="camera2Points">Массив двухмерных координат для второй камеры</param>
        /// <returns>Массив триангулированных трехмерных точек</returns>
        public static Mat TriangulatePointsInMat(CameraModel camera1, CameraModel camera2, Mat camera1Points, Mat camera2Points)
        {
            // применение триангуляции
            //Vec4d[] resCoord4D = Cv2.TriangulatePoints(camera1.ProjectionMat, camera2.ProjectionMat, camera1Points, camera2Points);
            Mat triangulatedPoints = new Mat();
            Cv2.TriangulatePoints(camera1.ProjectionMatrixMat, camera2.ProjectionMatrixMat, camera1Points, camera2Points, triangulatedPoints);

            // Избавляемся от вектора-столбца, который отвечает за масштабируемость
            // TODO: избавиться от двойного преобрзования в формат Mat??
            Cv2.ConvertPointsFromHomogeneous(triangulatedPoints.T(), triangulatedPoints);

            return triangulatedPoints;
        }

        /// <summary>
        /// Раасчитывает 3D ошибку репроекции между массивами трехмерных координат 
        /// </summary>
        /// <param name="referenceCoords3D">Массив исходных трехмерных координат</param>
        /// <param name="measuredCoords3D">Массив измеренных трехмерных координат</param>
        /// <returns>Массив трехмерных евклидовых расстояний между координатами</returns>
        public static double[] Calculate3DReprojectionError(IList<Point3d> referenceCoords3D, IList<Point3d> measuredCoords3D)
        {
            int length = measuredCoords3D.Count();

            if (length != referenceCoords3D.Count) 
            {
                throw new ArgumentException("Ошибка расчета ошибки 3D репроекции. Размерности referenceCoords3D и measuredCoords3D не совпадают.");
            }

            double[] error3D = new double[length];

            for (int i = 0; i < length; i++)
            {
                error3D[i] = Math.Sqrt(Math.Pow(referenceCoords3D[i].X - measuredCoords3D[i].X, 2) +
                                       Math.Pow(referenceCoords3D[i].Y - measuredCoords3D[i].Y, 2) +
                                       Math.Pow(referenceCoords3D[i].Z - measuredCoords3D[i].Z, 2));
            }

            return error3D;
        }

        /// <summary>
        /// Раасчитывает 2D ошибку репроекции для измеренных трехмерных координат для двух камер
        /// </summary>
        /// <param name="measuredCoords3D">Массив измеренных трехмерных координат</param>
        /// <param name="measuredCoords2D1">Массив измеренных двухмерных координат для первой камеры</param>
        /// <param name="measuredCoords2D2">Массив измеренных двухмерных координат для второй камеры</param>
        /// <param name="camera1">Первая камера для расчета</param>
        /// <param name="camera2">Вторая камера для расчета</param>
        /// <returns>Массивы двухмерных евклидовых расстояний между измеренными координатами и репроецированными для каждой камеры</returns>
        public static (double[], double[]) Calculate2DReprojectionError(IList<Point3d> measuredCoords3D, IList<Point2d> measuredCoords2D1, IList<Point2d> measuredCoords2D2, CameraModel camera1, CameraModel camera2)
        {
            int length = measuredCoords3D.Count();

            if (length != measuredCoords2D1.Count || length!= measuredCoords2D2.Count) 
            {
                throw new ArgumentException("Ошибка расчета ошибки 2D репроекции. Размерности measuredCoords3D и measuredCoords2D1 или measuredCoords2D2 не совпадают.");
            }

            Point2d[] camera1ProjectedCoords2D = camera1.ProjectPointsOnMatrix(measuredCoords3D);
            Point2d[] camera2ProjectedCoords2D = camera2.ProjectPointsOnMatrix(measuredCoords3D);

            double[] error2D1 = new double[length];
            double[] error2D2 = new double[length];

            for (int i = 0; i < length; i++)
            {
                error2D1[i] = Math.Sqrt(Math.Pow(camera1ProjectedCoords2D[i].X - measuredCoords2D1[i].X, 2) +
                                        Math.Pow(camera1ProjectedCoords2D[i].Y - measuredCoords2D1[i].Y, 2));

                error2D2[i] = Math.Sqrt(Math.Pow(camera2ProjectedCoords2D[i].X - measuredCoords2D2[i].X, 2) +
                                        Math.Pow(camera2ProjectedCoords2D[i].Y - measuredCoords2D2[i].Y, 2));
            }

            return (error2D1, error2D2);
        }

        /// <summary>
        /// Расчитывает стандратное отклонение в массиве чисел
        /// </summary>
        /// <param name="array">Массив чисел</param>
        /// <returns>Стандартное отклонение</returns>
        public static double Std(this IEnumerable<double> array)
        {
            int length = array.Count();
            double average = array.Average();

            return Math.Sqrt(array.Select(x => Math.Pow(x - average, 2)).Sum() / (length + 1));
        }

        public static void ShowMatrix(Mat matrix)
        {
            for (int i = 0; i < matrix.Rows; i++)
            {
                for (int j = 0; j < matrix.Cols; j++)
                {
                    if (j + 1 == matrix.Cols)
                        Console.WriteLine($" {matrix.At<double>(i, j):F5}");
                    else
                        Console.Write($" {matrix.At<double>(i, j):F5},");
                }
            }

            Console.WriteLine();
        }
    }
}
