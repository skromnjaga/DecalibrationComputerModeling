using System;
using System.IO;
using System.Linq;
using System.Drawing;
using System.Text.Json.Nodes;
using System.Collections.Generic;

using ScottPlot;
using OpenCvSharp;
using System.Reflection;

namespace DecalibrationComputerModeling;


class Program
{
    static void Main(string[] args)
    {
        ModelDecalibration();
        //LaunchDecalibrationIntrinsicParams();
    }

    static void ModelDecalibration()
    {
        // Задаем параметры стереосистемы
        double focalLength = 15;
        double sensorSizeX = 7.2;
        double sensorSizeY = 5.3;
        int sensorSizeInPixlesX = 2048;
        int sensorSizeInPixelsY = 1536;

        double[] distCoeffs = { 0.0, 0.0, 0.0, 0.0, 0.0 };

        // Положение второй камеры относительно первой
        double[,] rMatr2 =
            {
                {0.9780, 0.0298, 0.2065},
                {-0.0175, 0.9980, -0.0611},
                {-0.2079, 0.0561, 0.9765}
            };
        double[] tVec2 = { -315, 115, 40 };

        // Создаем камеры
        CameraModel camera1 = new CameraModel()
        {
            FocalLength = focalLength,
            SensorSize = new(sensorSizeX, sensorSizeY),
            SensorSizeInPixels = new(sensorSizeInPixlesX, sensorSizeInPixelsY),
        };

        // Вторая камера отнесена от первой согласно rMatr2 и tVec2
        CameraModel camera2 = new CameraModel()
        {
            FocalLength = focalLength,
            SensorSize = new(sensorSizeX, sensorSizeY),
            SensorSizeInPixels = new(sensorSizeInPixlesX, sensorSizeInPixelsY),
            RotMatrix = rMatr2,
            TransVector = tVec2
        };

        // Создаем 3D координаты моделируеомй поверхности
        Point3d[] pointsCoords3D = CreateFlatSurfacePointsCoords(1500, 300, 200, 45, 30, 30);

        // TODO: Вносим изменения в калибровочные параметры
        // ...

        // Проецируем 3D координаты на матрицы камер стереосистемы
        Point2d[] pointsCoords2DCamera1 = camera1.ProjectPointsOnMatrix(pointsCoords3D);
        Point2d[] pointsCoords2DCamera2 = camera2.ProjectPointsOnMatrix(pointsCoords3D);

        // Рассчитываем триангуляцию
        Point3d[] triangulatedPoints = Helper.TriangulatePoints(camera1, camera2, pointsCoords2DCamera1, pointsCoords2DCamera2);

        // Рассчитываем ошибку репроекции
        double[] error3D = Helper.Calculate3DReprojectionError(pointsCoords3D, triangulatedPoints);

        (double[] error2D1, double[] error2D2) = Helper.Calculate2DReprojectionError(triangulatedPoints, pointsCoords2DCamera1, pointsCoords2DCamera2, camera1, camera2);

        Console.WriteLine($"Средняя 3D ошибка репроекции = {error3D.Average()}, стандартное отклонение = {error3D.Std()}");
        Console.WriteLine($"Средняя 2D ошибка репроекции #1 = {error2D1.Average()}, стандартное отклонение = {error2D1.Std()}");
        Console.WriteLine($"Средняя 2D ошибка репроекции #2 = {error2D2.Average()}, стандартное отклонение = {error2D2.Std()}");
    }

    /// <summary>
    /// Создает массив трехмерных точек плоской поверхности (модель крыла) с заданными параметрами
    /// </summary>
    /// <param name="z0">Расстояние до крыла</param>
    /// <param name="width">Ширина плоскости</param>
    /// <param name="height">Длина плоскости</param>
    /// <param name="tetta">Угол наклона плоскости</param>
    /// <param name="stepX">Шаг разбиения на точки по горизонтали</param>
    /// <param name="stepY">Шаг разбиения на точки по вертикали</param>
    /// <returns>Массив трехмерных координат плоскости</returns>
    static Point3d[] CreateFlatSurfacePointsCoords(double z0, int width, int height, double tetta, int stepX, int stepY)
    {
        int xPointsNum = width / stepX + 1;
        int yPointsNum = height / stepY + 1;

        Point3d[] matOfPoints = new Point3d[xPointsNum * yPointsNum];

        int p = 0;

        for (int i = 0; i < xPointsNum; i++)
        {
            for (int j = 0; j < yPointsNum; j++)
            {
                matOfPoints[p].X = i * width / (xPointsNum - 1.0) - width / 2.0;
                matOfPoints[p].Y = j * height / (yPointsNum - 1.0) - height / 2.0;
                matOfPoints[p].Z = Math.Tan(tetta * Math.PI / 180) * matOfPoints[p].Y + z0;

                p++;
            }
        }

        return matOfPoints;
    }
}