using System;
using System.Linq;
using System.Collections.Generic;

using OpenCvSharp;

namespace DecalibrationComputerModeling;


/// <summary>
/// Реализует механизм работы с камерой по pinhole модели 
/// </summary>
public class CameraModel
{
    /// <summary>
    /// Фокусное расстояние объектива
    /// </summary>
    public double FocalLength { get; set; }
    
    /// <summary>
    /// Размер матрицы камеры в мм
    /// </summary>
    public (double SizeX, double SizeY) SensorSize { get; set; }
    
    /// <summary>
    /// Размер матрицы камеры в пикселях
    /// </summary>
    public (int SizeX, int SizeY) SensorSizeInPixels { get; set; }

    public double[,] _intrinsicMatrix = default!;

    /// <summary>
    /// Матрица внутренних параметров
    /// </summary>
    public double[,] IntrinsicMatrix
    {
        get
        {
            // Расчет матрицы идет после указания размера сенсора, т.е. после вызова конструктора
            if (_intrinsicMatrix is null)
            {
                _intrinsicMatrix = CalcIntrinsicParams();
                IntrinsicMatrixMat = Mat.FromArray(_intrinsicMatrix);
            }

            return _intrinsicMatrix;
        }
        set
        { 
            _intrinsicMatrix = value;
            IntrinsicMatrixMat = Mat.FromArray(_intrinsicMatrix);
        }
    }

    protected Mat _intrinsicMatrixMat = default!;

    /// <summary>
    /// Матрица внутренних параметров в формате Mat
    /// </summary>
    public Mat IntrinsicMatrixMat 
    {
        get
        {
            // Расчет матрицы идет после указания размера сенсора, т.е. после вызова конструктора
            if (_intrinsicMatrix is null)
            {
                _intrinsicMatrix = CalcIntrinsicParams();
                _intrinsicMatrixMat = Mat.FromArray(_intrinsicMatrix);
            }

            return _intrinsicMatrixMat;
        }
        private set
        {
            _intrinsicMatrixMat = value;
        }
    }

    /// <summary>
    /// Углы Эйлера описывающие положение камеры в мировой системе координат
    /// </summary>
    public double[] EulerZYX 
    { 
        get
        {
            throw new NotImplementedException();
        }
        set
        {
            throw new NotImplementedException();
        }
    }

    private double[,] _rotMatrix = default!;

    /// <summary>
    /// Матрица вращения
    /// </summary>
    public double[,] RotMatrix 
    { 
        get => _rotMatrix;
        set
        {
            _rotMatrix = value;
            RotMatrixMat = Mat.FromArray(_rotMatrix);
        }
    }

    /// <summary>
    /// Матрица вращения в формате Mat
    /// </summary>
    public Mat RotMatrixMat { get; private set; } = default!;

    private double[] _transVector = default!;

    /// <summary>
    /// Вектор смещения
    /// </summary>
    public double[] TransVector 
    {
        get => _transVector;
        set
        {
            _transVector = value;
            TransVectorMat = Mat.FromArray(_transVector);
        }
    }

    /// <summary>
    /// Вектор смещения в формате Mat
    /// </summary>
    public Mat TransVectorMat { get; private set; } = default!;

    /// <summary>
    /// Проективаня матрица камеры. Возращает значение, полученное
    /// путем преобразования ProjectionMatrixMat в double[,].
    /// </summary>
    public double[,] ProjectionMatrix
    {
        get
        {
            ProjectionMatrixMat.GetRectangularArray(out double[,] projMatrix);
            return projMatrix;
        }
    }

    /// <summary>
    /// Проективаня матрица в формате Mat. Возращает значение путем перемножения
    /// IntrinsicMatrixMat на совмещенную матрицу RotMatrixMat с TransVectorMat.
    /// </summary>
    public Mat ProjectionMatrixMat => CalcMatrixOfProjection();

    private double[] _distCoeffs = default!;

    /// <summary>
    /// Коэффициенты дисторсии
    /// </summary>
    public double[] DistCoeffs
    {
        get => _distCoeffs;
        set
        {
            _distCoeffs = value;
            DistCoeffsMat = Mat.FromArray(_distCoeffs);
        }
    }

    /// <summary>
    /// Коэффициенты дисторсии в формате Mat
    /// </summary>
    public Mat DistCoeffsMat { get; private set; } = default!;

    /// <summary>
    /// Матрица точек проекции на матрицу камеры
    /// </summary>
    public Mat CoordOnScreen2D { get; set; } = new();


    public CameraModel()
    {
        // Инициализируем параметры камеры значениями по умолчанию
        DistCoeffs = new double[5];
        RotMatrix = new double[3, 3]
            {
                {1, 0, 0 },
                {0, 1, 0 },
                {0, 0, 1 },
            };
        TransVector = new double[3];
    }

    /// <summary>
    /// Инициализирует матрицу внутренних параметров камеры
    /// </summary>
    /// <returns>Матрицу внутренних параметров камеры</returns>
    private double[,] CalcIntrinsicParams()
    {
        double[,] intrinsicMatrix;

        // Проверка на деление на 0
        if ((SensorSize.SizeX != 0) & (SensorSize.SizeY != 0) & (FocalLength != 0))
        {
            intrinsicMatrix = new double[3, 3]
            {
                { SensorSizeInPixels.SizeX / SensorSize.SizeX * FocalLength, 0,  SensorSizeInPixels.SizeX / 2},
                { 0, SensorSizeInPixels.SizeY / SensorSize.SizeY * FocalLength, SensorSizeInPixels.SizeY / 2},
                { 0, 0, 1}
            };
        }
        else
        {
            throw new ArgumentException("Невозможно инициализировать матрицу внутренних параметров, размер матрицы или фокусное расстояние равно 0.");
        }

        return intrinsicMatrix;
    }

    /// <summary>
    /// Расчитывает матрицу проекции путем умножения матрицы внутренних параметров IntrinsicParamsMat
    /// на совмещенную матрицу поворота и переноса
    /// </summary>
    /// <returns>Матрицу проекции</returns>
    private Mat CalcMatrixOfProjection()
    {
        Mat projectionMatrixMat = new Mat();
        Cv2.HConcat(RotMatrixMat, TransVectorMat, projectionMatrixMat);

        projectionMatrixMat = IntrinsicMatrixMat * projectionMatrixMat;

        return projectionMatrixMat;
    }
    
    /// <summary>
    /// Проецирует трехмерные координаты в двухмерные кординаты на матрицу камеры
    /// </summary>
    /// <param name="pointsCoords3D">Массив трехмерных координат в формате Mat</param>
    /// <returns>Массив двухмерных координат на матрице</returns>
    public Point2d[] ProjectPointsOnMatrix(Mat pointsCoords3D)
    {
        Mat coordOnScreen2D = new Mat();
        Cv2.ProjectPoints(pointsCoords3D, RotMatrixMat, TransVectorMat, IntrinsicMatrixMat, DistCoeffsMat, coordOnScreen2D);
        coordOnScreen2D.GetArray(out Point2d[] result);

        return result;
    }

    /// <summary>
    /// Проецирует трехмерные координаты в двухмерные кординаты на матрицу камеры
    /// </summary>
    /// <param name="pointsCoords3D">Массив трехмерных координат</param>
    /// <returns>Массив двухмерных координат на матрице</returns>
    public Point2d[] ProjectPointsOnMatrix(IEnumerable<Point3d> pointsCoords3D)
    {
        return ProjectPointsOnMatrix(Mat.FromArray(pointsCoords3D));
    }

    /// <summary>
    /// Проецирует трехмерные координаты в двухмерные кординаты на матрицу камеры в формате Mat
    /// </summary>
    /// <param name="pointsCoords3D">Массив трехмерных координат в формате Mat</param>
    /// <returns>Массив двухмерных координат на матрице</returns>
    public Mat ProjectPointsOnMatrixInMat(Mat pointsCoords3D)
    {
        Mat coordOnScreen2D = new Mat();
        Cv2.ProjectPoints(pointsCoords3D, RotMatrixMat, TransVectorMat, IntrinsicMatrixMat, DistCoeffsMat, coordOnScreen2D);

        return coordOnScreen2D;
    }
}