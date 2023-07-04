using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Factorization;
using TMPro;

public class CoordinateAlignment : MonoBehaviour
{

    /**
     * 
     * Unity Fields
     * 
     **/

    [SerializeField] private GameObject ovrCamera;

    [SerializeField] private TextMeshProUGUI tmp;

    [SerializeField] private SixenseHands rightHand;
    [SerializeField] private SixenseHands leftHand;
    
    [SerializeField] private OVRSkeleton leftHandSkeleton;
    [SerializeField] private OVRSkeleton rightHandSkeleton;

    [SerializeField] private GameObject calibratedHydraRight;
    [SerializeField] private GameObject handCenter;

    SixenseInput.Controller rightController;
    SixenseInput.Controller leftController;


    /**
     * 
     * Calibration Variables
     *  
     **/

    public static int CALIBRATION_SIZE = 100;
    private int calibrationCount;
    private bool calibrated = false;

    private Vector3[] virtualPositions;
    private Vector3[] physicalPositions;

    private Matrix4x4 transformationMatrix;



    private void Start() 
    {
        virtualPositions = new Vector3[CALIBRATION_SIZE];
        physicalPositions = new Vector3[CALIBRATION_SIZE];

    }


    private void Update() {
        rightController = SixenseInput.GetController( rightHand );
        leftController = SixenseInput.GetController( leftHand );

        if ((rightController != null && rightController.Enabled) &&  
            (leftController != null && leftController.Enabled))
        {

            if (calibrated) {
                ShowMappedPosition();

            } else {
                
                Calibrate();

            }

            handCenter.transform.position = GetHandCentroid(rightHandSkeleton);
        }
    }


    private void Calibrate() {
        
        if (calibrationCount == CALIBRATION_SIZE) {    
            transformationMatrix = ComputeTransformationMatrix();
            calibrated = true;

        } else {
            AddPointsToCalibration();
    
        }
    }

    private void AddPointsToCalibration() {

        Debug.Log(physicalPositions + "\n" + virtualPositions);
        tmp.text = calibrationCount + ", " + CALIBRATION_SIZE;

        if (ovrCamera.GetComponent<HandTrackerCenter>().GetTrackedPosition() != Vector3.zero) {

            if (rightController.Trigger == 1) {

                Vector3 virtualPos = GetHandCentroid(rightHandSkeleton);
                Vector3 physicalPos = rightController.Position;
                

                float distance = 9999999;

                if (calibrationCount > 0) {

                    distance = Vector3.Distance(virtualPos, virtualPositions[calibrationCount-1]);
                
                }

                if (distance > 0.10f) {

                    virtualPositions[calibrationCount] = virtualPos;
                    physicalPositions[calibrationCount] = physicalPos;



                    calibrationCount++;
                }
                
            } 
        }
    }

  
    Vector3 GetHandCentroid(OVRSkeleton hand) {

        int numberOfBones = hand.GetCurrentNumBones();

        float totalX = 0;
        float totalY = 0;
        float totalZ = 0;

        for (int i = 0; i < numberOfBones; i++) {
            totalX += hand.GetComponent<OVRSkeleton>().Bones[i].Transform.position.x;
            totalY += hand.GetComponent<OVRSkeleton>().Bones[i].Transform.position.y;
            totalZ += hand.GetComponent<OVRSkeleton>().Bones[i].Transform.position.z;
            

        }

        return new Vector3(totalX/numberOfBones, totalY/numberOfBones, totalZ/numberOfBones);

    }


    private void ShowMappedPosition()
    {

        Vector3 newPhysicalPosition = rightController.Position; 
        Vector3 mappedPosition = ApplyTransformation(transformationMatrix, newPhysicalPosition);
        
        calibratedHydraRight.transform.position = mappedPosition;
        calibratedHydraRight.transform.rotation = rightController.Rotation;
        tmp.text = "Physical Position: " + newPhysicalPosition + "\nMapped Position: " + mappedPosition;
    }


    private Matrix4x4 ComputeTransformationMatrix()
    {
        // Step 2: Compute the Transformation Matrix
        Vector3 virtualCentroid = CalculateCentroid(virtualPositions);
        Vector3 physicalCentroid = CalculateCentroid(physicalPositions);

        Matrix4x4 covarianceMatrix = CalculateCovarianceMatrix(virtualPositions, physicalPositions, virtualCentroid, physicalCentroid);

        Matrix4x4 rotationMatrix = CalculateRotationMatrix(covarianceMatrix);

        Vector3 translationVector = CalculateTranslationVector(rotationMatrix, virtualCentroid, physicalCentroid);

        Matrix4x4 localTransformationMatrix = CreateTransformationMatrix(rotationMatrix, translationVector);

        float scaleFactor = CalculateScaleFactor(virtualPositions, physicalPositions);


        return ApplyScaleFactor(scaleFactor,localTransformationMatrix);
    }



    private Vector3 CalculateCentroid(Vector3[] positions)
    {
        Vector3 sum = Vector3.zero;
        foreach (Vector3 position in positions)
        {
            sum += position;
        }
        return sum / positions.Length;
    }

    private Matrix4x4 CalculateCovarianceMatrix(Vector3[] virtualPositions, Vector3[] physicalPositions, Vector3 virtualCentroid, Vector3 physicalCentroid)
    {
        Matrix4x4 covarianceMatrix = new Matrix4x4();

        for (int i = 0; i < virtualPositions.Length; i++)
        {
            Vector3 virtualPosition = virtualPositions[i] - virtualCentroid;
            Vector3 physicalPosition = physicalPositions[i] - physicalCentroid;

            Matrix4x4 outerProduct = Matrix4x4OuterProduct(virtualPosition, physicalPosition);
            Matrix4x4 newCovarianceMatrix = Matrix4x4Addition(covarianceMatrix, outerProduct);
            covarianceMatrix = newCovarianceMatrix;
        }

        return covarianceMatrix;
    }

    private float CalculateScaleFactor(Vector3[] virtualPoints, Vector3[] physicalPoints)
    {
        
        float totalVirtualDistance = 0f;
        float totalPhysicalDistance = 0f;

        for (int i = 0; i < virtualPoints.Length; i++)
        {
            float virtualDistance = Vector3.Distance(virtualPoints[i], virtualPoints[(i + 1) % virtualPoints.Length]);
            float physicalDistance = Vector3.Distance(physicalPoints[i], physicalPoints[(i + 1) % physicalPoints.Length]);

            totalVirtualDistance += virtualDistance;
            totalPhysicalDistance += physicalDistance;
        }

        float scaleFactor = totalVirtualDistance / totalPhysicalDistance;

        return scaleFactor;
    }

    private Matrix4x4 ApplyScaleFactor(float scaleFactor, Matrix4x4 originalTransformationMatrix) {

        Matrix4x4 scaleMatrix = Matrix4x4.identity;
        scaleMatrix.m00 = scaleFactor; // X scale
        scaleMatrix.m11 = scaleFactor; // Y scale
        scaleMatrix.m22 = scaleFactor; // Z scale
    
        Matrix4x4 newTransformationMatrix = scaleMatrix * originalTransformationMatrix;
        return newTransformationMatrix;
    }


    private Matrix4x4 Matrix4x4OuterProduct(Vector3 a, Vector3 b)
    {
        Matrix4x4 result = new Matrix4x4();
        result.SetColumn(0, a.x * b);
        result.SetColumn(1, a.y * b);
        result.SetColumn(2, a.z * b);
        result.SetColumn(3, Vector4.zero);
        return result;
    }

    Matrix4x4 Matrix4x4Addition(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 result = new Matrix4x4();

        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                result[row, col] = a[row, col] + b[row, col];
            }
        }

        return result;
    }

    private Matrix4x4 CalculateRotationMatrix(Matrix4x4 covarianceMatrixUnity)
    {
        // Use SVD or any other suitable method to calculate the rotation matrix
        Matrix<double> covarianceMatrix = ConvertToMathNetMatrix(covarianceMatrixUnity);
        Svd<double> svd = covarianceMatrix.Svd(true);
        Matrix<double> rotationMatrix = svd.U * svd.VT;

        // if (rotationMatrix.Determinant() < 0)
        // {
        //     // Change the sign of the last column of the rotation matrix
        //     rotationMatrix.SetColumn(2, -rotationMatrix.Column(2));
        // }

        return ConvertToUnityMatrix(rotationMatrix);
    }

    public static Matrix<double> ConvertToMathNetMatrix(Matrix4x4 matrix)
    {
        double[,] data = new double[4, 4];

        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                data[row, col] = matrix[row, col];
            }
        }

        return DenseMatrix.OfArray(data);
    }

    public static Matrix4x4 ConvertToUnityMatrix(Matrix<double> matrix)
    {
        if (matrix.RowCount != 4 || matrix.ColumnCount != 4)
        {
            Debug.LogError("Invalid matrix dimensions. Cannot convert to Unity Matrix4x4.");
            return Matrix4x4.identity;
        }

        Matrix4x4 unityMatrix = new Matrix4x4();
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                unityMatrix[row, col] = (float)matrix[row, col];
            }
        }

        return unityMatrix;
    }

    private Vector3 CalculateTranslationVector(Matrix4x4 rotationMatrix, Vector3 virtualCentroid, Vector3 physicalCentroid)
    {
        Vector3 translationVector = virtualCentroid - rotationMatrix.MultiplyPoint3x4(physicalCentroid);
        return translationVector;
    }

    private Matrix4x4 CreateTransformationMatrix(Matrix4x4 rotationMatrix, Vector3 translationVector)
    {
        Matrix4x4 transformationMatrix = Matrix4x4.identity;
        transformationMatrix.SetTRS(Vector3.zero, Quaternion.identity, Vector3.one);
        transformationMatrix.SetColumn(3, new Vector4(translationVector.x, translationVector.y, translationVector.z, 1f));
        transformationMatrix = rotationMatrix * transformationMatrix;
        return transformationMatrix;
    }

    private Vector3 ApplyTransformation(Matrix4x4 transformationMatrix, Vector3 physicalPosition)
    {
        Vector3 mappedPosition = transformationMatrix.MultiplyPoint3x4(physicalPosition);
        return mappedPosition;
    }
}