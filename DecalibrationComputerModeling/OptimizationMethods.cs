using System;
using System.Linq;

using OpenCvSharp;

namespace DecalibrationComputerModeling;


public class OptimizationMethods
{
    /// <summary>
    /// Минимизирует ошибку репроекции для стереосистемы камер путем подбора внутренних и внешних параметров
    /// </summary>
    /// <param name="camera1">Первая камера стереосистемы</param>
    /// <param name="camera2">Вторая камера стереосистемы</param>
    /// <param name="coords2DCamera1">Массив двухмерных точек для первой камеры</param>
    /// <param name="coords2DCamera2">Массив двухмерных точек для второй камеры</param>
    /// <param name="vec0">Вектор начальных значений оптимизируемых параметров</param>
    /// <returns>Оптимизированные значения параметров</returns>
    public static double[] NelderMeadReprojectionErrorMinimizing(CameraModel camera1, CameraModel camera2, Mat coords2DCamera1, Mat coords2DCamera2, double[] vec0)
    {
        // Количество оптимизируемых параметров
        int N = 6;

        // Количество попыток поиска минимума
        int MN = N * 1;

        // Максимальное количество итераций для одной попытки
        int itersMaxNumber = 500;

        // Максимальное отклонение вектора переноса
        double tMax = 10.0;
        // Максимальное отклонение углов поворота Эйлера
        double rMax = 2 * Math.PI / 180;

        // Вектор оптимизируемых параметров
        double[] paramsVectorInitial = (double[])vec0.Clone();

        // Минимальное значение ошибки, при которой минимум считается найденным
        double deltaErr = Math.Pow(10, -6);

        // Результаты оптимизации для всех попыток
        double[][] paramsVectorBest = new double[MN][];
        for (int i = 0; i < MN; i++)
        {
            paramsVectorBest[i] = new double[N];
        }

        // Ошибки в попытках
        double[] errorVectorBest = new double[MN];

        // Коэффициент отражения
        double alpha = 1.0;
        // Коэффициент сжатия
        double beta = 0.5;
        // Коэффициент растяжения
        double gamma = 2.0;
        // Коэффициент глобального сжатия
        double sigma = 0.5;

        for (int m = 0; m < MN; m++)
        {
            // Симплекс для текущей попытки
            double[][] paramsVector = new double[N + 1][];

            // Случайная инициализация начальных значений параметров для второго и последущих проходов
            if (m > 0)
            {
                Random rnd = new Random();
                for (int i = 0; i < N; i++)
                {
                    if (i < 3) paramsVectorInitial[i] = vec0[i] + (2 * rnd.NextDouble() - 1) * (tMax / 2);
                    else paramsVectorInitial[i] = vec0[i] + (2 * rnd.NextDouble() - 1) * (rMax / 2);
                }
            }

            // Копируем первую вершину симплекса из начальных значений параметров
            paramsVector[0] = (double[])paramsVectorInitial.Clone();
            // Инициализируем остальные веришины симплекса пустыми массивами
            for (int i = 1; i < N + 1; i++)
            {
                paramsVector[i] = new double[N];
            }

            double stepT = 0.5;
            double stepR = 0.05 * Math.PI / 180;

            double dt1 = stepT * (Math.Sqrt(N + 1) - 1) / (N * Math.Sqrt(2));
            double dt2 = stepT * (Math.Sqrt(N + 1) + N - 1) / (N * Math.Sqrt(2));

            double dr1 = stepR * (Math.Sqrt(N + 1) - 1) / (N * Math.Sqrt(2));
            double dr2 = stepR * (Math.Sqrt(N + 1) + N - 1) / (N * Math.Sqrt(2));

            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < N; j++)
                {
                    if (j < 3)
                    {
                        if (i != j) paramsVector[i + 1][j] = paramsVectorInitial[j] + dt1;
                        else paramsVector[i + 1][j] = paramsVectorInitial[j] + dt2;
                    }
                    else
                    {
                        if (i != j) paramsVector[i + 1][j] = paramsVectorInitial[j] + dr1;
                        else paramsVector[i + 1][j] = paramsVectorInitial[j] + dr2;
                    }
                }
            }

            // Значение функции в вершинах симплекса
            double[] errorVector = new double[N + 1];

            // Рассчитываем ошибку репроекции для вершин симплекса
            for (int i = 0; i < N + 1; i++)
            {
                errorVector[i] = CalcReprojectionErrorMod(coords2DCamera1, coords2DCamera2, camera1, camera2, vec0, paramsVector[i]);
            }

            // Начинаем итерации для поиска минимума
            for (int k = 0; k < itersMaxNumber; k++)
            {
                // Сортируем значения в вершинах симплекса по возрастанию
                double[] sortedErrorVector = errorVector.Order().ToArray();
                // Определяем индексы для вершины с максимальной, меньше максимальной и минимальной ошибкой
                int indexWorstError = Array.IndexOf(errorVector, sortedErrorVector[N]);
                int indexGoodError = Array.IndexOf(errorVector, sortedErrorVector[N - 1]);
                int indexBestError = Array.IndexOf(errorVector, sortedErrorVector[0]);

                // Рассчитываем центройд для вершин симплекса кроме вершины с максимальной ошибкой
                double[] paramsCentroid = new double[N];
                for (int j = 0; j < N; j++)
                {
                    double s = 0.0;
                    for (int i = 0; i < N + 1; i++)
                    {
                        if (i != indexWorstError)
                        {
                            s += paramsVector[i][j];
                        }
                    }

                    paramsCentroid[j] = s / N;
                }

                // Отражение
                double[] paramsReflected = new double[N];
                for (int i = 0; i < N; i++)
                {
                    paramsReflected[i] = (1 + alpha) * paramsCentroid[i] - alpha * paramsVector[indexWorstError][i];
                }

                // Рассчитываем значение функции в отраженной точке
                double errorReflected = CalcReprojectionErrorMod(coords2DCamera1, coords2DCamera2, camera1, camera2, vec0, paramsReflected);

                if (errorReflected < errorVector[indexBestError])
                {
                    // Рассчитываем значение функции в точке при растяжении
                    double[] paramsExpansioned = new double[N].Select((_, j) => gamma * paramsReflected[j] + (1 - gamma) * paramsCentroid[j]).ToArray();
                    // Расчет ошибки в этой точке 
                    double errorExpansioned = CalcReprojectionErrorMod(coords2DCamera1, coords2DCamera2, camera1, camera2, vec0, paramsExpansioned);

                    if (errorExpansioned < errorVector[indexBestError])
                    {
                        paramsVector[indexWorstError] = paramsExpansioned;
                        errorVector[indexWorstError] = errorExpansioned;
                    }
                    else
                    {
                        paramsVector[indexWorstError] = paramsReflected;
                        errorVector[indexWorstError] = errorReflected;
                    }
                }
                else
                {
                    if (errorReflected < errorVector[indexGoodError])
                    {
                        paramsVector[indexWorstError] = paramsReflected;
                        errorVector[indexWorstError] = errorReflected;
                    }
                    else
                    {
                        if (errorReflected < errorVector[indexWorstError])
                        {
                            paramsVector[indexWorstError] = paramsReflected;
                            errorVector[indexWorstError] = errorReflected;
                        }

                        // Рассчитываем значение функции в точке при сжатии
                        double[] paramsContractioned = new double[N].Select((x, j) => beta * paramsVector[indexWorstError][j] + (1 - beta) * paramsCentroid[j]).ToArray();
                        // Расчет ошибки в этой точке
                        double errorContractioned = CalcReprojectionErrorMod(coords2DCamera1, coords2DCamera2, camera1, camera2, vec0, paramsContractioned);

                        if (errorContractioned < errorVector[indexWorstError])
                        {
                            paramsVector[indexWorstError] = paramsContractioned;
                            errorVector[indexWorstError] = errorContractioned;
                        }
                        else
                        {
                            // Глобальное сжатие ??
                            for (int i = 0; i < N + 1; i++)
                            {
                                for (int j = 0; j < N; j++)
                                {
                                    // TODO: Странная формула, в вики она задана в другом виде
                                    paramsVector[i][j] = sigma * (paramsVector[i][j] + paramsVector[indexBestError][j]);
                                }
                            }

                            for (int i = 0; i < N + 1; i++)
                            {
                                errorVector[i] = CalcReprojectionErrorMod(coords2DCamera1, coords2DCamera2, camera1, camera2, vec0, paramsVector[i]);
                            }
                        }
                    }
                }

                // Проверка на "сходимость"
                if (StandardDeviation(errorVector) < deltaErr)
                {
                    // Переход к следующей попытке
                    break;
                }
            }

            double errMin = errorVector.Min();
            int indErrMin = Array.IndexOf(errorVector, errMin);
            paramsVectorBest[m] = paramsVector[indErrMin];
            errorVectorBest[m] = errMin;
        }

        int indBestErrMin = Array.IndexOf(errorVectorBest, errorVectorBest.Min());
        return paramsVectorBest[indBestErrMin];
    }

    /// <summary>
    /// Рассчитывает ошибку репроекции для стереосистемы камер с учетом штрафной функции
    /// </summary>
    /// <param name="coords2DCamera1">Массив двухмерных координат для первой камеры</param>
    /// <param name="coords2DCamera2">Массив двухмерных координат для второй камеры</param>
    /// <param name="camera1">Первая камера</param>
    /// <param name="camera2">Вторая камера</param>
    /// <param name="oldVecP">Исходный набор параметров камер</param>
    /// <param name="newVecP">Оптимизированный набор параметров камер</param>
    /// <returns>Среднее значение ошибки репроекции с учетом штрафной функции</returns>
    private static double CalcReprojectionErrorMod(Mat coords2DCamera1, Mat coords2DCamera2, 
                                                   CameraModel camera1, CameraModel camera2, 
                                                   double[] oldVecP, double[] newVecP)
    {
        // Меняем параметры в камерах на новые значения
        double[] a = { newVecP[5], newVecP[4], newVecP[3] };
        //camera2.SetEulerZYXVector((double[])a.Clone());
        
        //double[,] t = { { newVecP[0] }, { newVecP[1] }, { newVecP[2] } };
        //camera2.SetTranslationVector((double[,])t.Clone());
        double[] translationVectorRecalibrated = new double[3] { newVecP[0], newVecP[1], newVecP[2] };
        camera1.TransVector = translationVectorRecalibrated;

        // Расчитываем триангуляцию с новыми параметрами камер
        Mat coords3DRecalibrated = Helper.TriangulatePointsInMat(camera1, camera2, coords2DCamera1, coords2DCamera2);

        // Рассчитываем репроекцию триангулированных точек
        Mat coords2DCamera1Recalibrated = camera1.ProjectPointsOnMatrixInMat(coords3DRecalibrated);
        Mat coords2DCamera2Recalibrated = camera2.ProjectPointsOnMatrixInMat(coords3DRecalibrated);
        
        return ExtMathFuncs.ReprojectionErrorMod(coords2DCamera1, coords2DCamera2, coords2DCamera1Recalibrated, 
                                                 coords2DCamera2Recalibrated, oldVecP, newVecP);
    }

    /// <summary>
    /// Расчситывает стандартное отклонение для заданного массива значений. Для расчета 
    /// среднего и стандартного отклонения используется длина массива + 1.
    /// </summary>
    /// <param name="array">Массив значений</param>
    /// <returns>Стандартное отклонение значений в массиве</returns>
    private static double StandardDeviation(double[] array)
    {
        int N = array.Length;

        double mean = array.Sum() / (N + 1);
        return Math.Sqrt(array.Select(x => Math.Pow(x - mean, 2)).Sum() / (N + 1));
    }
}