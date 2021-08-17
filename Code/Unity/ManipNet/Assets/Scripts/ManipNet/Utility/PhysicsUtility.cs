using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Accord.Statistics.Models.Regression.Fitting;
using Accord.Statistics.Models.Regression.Linear;
using Accord.Math.Optimization.Losses;

public static class PhysicsUtility
{
    // ===================================== for physics evaluation of contact ========================================
    public static Vector3[] Get4LocalBasis(float mu){
        /*
        here gets the local directions where we say the norm is the Y asix, later Y axis need to be the contact norm
        coefficient mu need to be bigger than 0
        big value means wider coin and and more potential solution
        usual material seems to be 0.3
        */ 
        Vector3[] basisDirections = new Vector3[4];
        if(mu<=0){
            Debug.Log("friction coefficient need to be bigger than 0, but now is " + mu + " , now change to 0.01");
            mu = 0.01f;
        }
        basisDirections[0] = new Vector3(1f,1f/mu,0f);
        basisDirections[1] = new Vector3(-1f,1f/mu,0f);
        basisDirections[2] = new Vector3(0,1f/mu,1f);
        basisDirections[3] = new Vector3(0,1f/mu,-1f);
        return basisDirections;
    }

    public static Vector3[] Get4GlobalBasis(float mu, Vector3 contactNorm){
        Vector3[] localDirections = Get4LocalBasis(mu);
        //todo here is important to flip
        contactNorm = -contactNorm;
        Quaternion rotation = Quaternion.FromToRotation(Vector3.up, contactNorm);
        for(int i=0; i<localDirections.Length; i++){
            localDirections[i] = rotation * localDirections[i];
        }
        return  localDirections;
    }

    public static Vector3[] Get4GlobalBasisForSlidingFriction(float mu, Vector3 contactNorm, Vector3 contactVelocity){
        // here we still keep 4 basis and set 3 tobe vector.zero
        Vector3[] basis = new Vector3[4];
        for(int i=0; i<basis.Length; i++){
            basis[i] = Vector3.zero;
        }
        Vector3 supportForce = contactNorm.normalized/mu;
        Vector3 slidingFriction = -contactVelocity.normalized;
        Vector3 force = supportForce + slidingFriction;

        // flip the force
        basis[0] = -force;
        // here 
        return basis;
    }

    public static Vector3[] GetModified4GlobalBasisForL_dot(Vector3[] globalBasis, Vector3 contactPositionRelativeToCOM){
        // [r(t)-x(t)]F(t), [] means skew symmetric
        Vector3[] modifiedBasis = new Vector3[globalBasis.Length];
        Matrix4x4 rx = GetSkewMatrix(contactPositionRelativeToCOM);
        for(int i=0; i<globalBasis.Length; i++){
            modifiedBasis[i] = rx.MultiplyVector(globalBasis[i]);
        }
        return modifiedBasis;
    }

    public static Matrix4x4 GetSkewMatrix(Vector3 vec){
        Matrix4x4 skewMatrix = Matrix4x4.zero;
        skewMatrix[0,1] = -vec.z; 
        skewMatrix[0,2] = vec.y;
        skewMatrix[1,0] = vec.z;
        skewMatrix[1,2] = -vec.x;
        skewMatrix[2,0] = -vec.y;
        skewMatrix[2,1] = vec.x;
        return skewMatrix;
    }

    public static float GetNNLSLossExample(){
        var inputs = new double[][]
        {
            new[] { 1.0, 1.0, 1.0 },
            new[] { 2.0, 4.0, 8.0 },
            new[] { 3.0, 9.0, 27.0 },
            new[] { 4.0, 16.0, 64.0 },
        };
        var outputs = new double[] { 0.23, 1.24, 3.81, 8.72 };
        // Create a NN LS learning algorithm
        var nnls = new NonNegativeLeastSquares()
        {
            MaxIterations = 100
        };
        // Use the algorithm to learn a multiple linear regression
        MultipleLinearRegression regression = nnls.Learn(inputs, outputs);
        // None of the regression coefficients should be negative:
        double[] coefficients = regression.Weights; // should be
        // Check the quality of the regression:
        double[] prediction = regression.Transform(inputs);
        float error = (float)new SquareLoss(expected: outputs).Loss(actual: prediction); // should be 0
        return error;
    } 

    public static float GetNNLSLoss(List<Vector3[]> linearBasis, List<Vector3[]> angularBasis,  Vector3 linearDynamic, Vector3 angularDynamic){
        /// <summary>
        /// inputs are force matrix, outputs are dynamics vectors
        /// linearBasis/angularBasis N contacts, 4 basis (N*4*3)
        /// linearDynamic/angularDynamic N contacts (N*3)
        /// apply min ||Ax-b||, s.t. x>=0
        /// </summary>
        /// 
        /// 
        // add gravity 
        Vector3 gravity = new Vector3(0f, -9.8f/120f, 0f);
        linearDynamic = linearDynamic - gravity;

        if(linearBasis.Count!=angularBasis.Count){
            Debug.Log("linear basis matrix not match angular basis matrix");
            return 0;
        }
        int nContact = linearBasis.Count;
        if(nContact==0){
            // if no contact happens, then just calculate the loss from the dynamics
            double[] dynamics = new double[6];
            dynamics[0] = linearDynamic.x;
            dynamics[1] = linearDynamic.y;
            dynamics[2] = linearDynamic.z;
            dynamics[3] = angularDynamic.x;
            dynamics[4] = angularDynamic.y;
            dynamics[5] = angularDynamic.z;
            double[] forces = new double[6]; 
            // no contact thus no force
            for(int i=0; i<6; i++){
                forces[i]=0f;
            }
            return (float)new SquareLoss(expected: dynamics).Loss(actual: forces); 
        }
        // if contact happens, then we do optimization to get the loss
        int nBasis = linearBasis[0].Length; // here actually is 4
        // init the A and b
        double[][] inputs = new double[6][];
        for(int i=0; i<6; i++){
            inputs[i] = new double[nBasis*nContact];
        }
        double[] outputs = new double[6];
        {// assign values for inputs
            for(int i=0; i<nContact; i++){
                Vector3[] linearB = linearBasis[i];   // 4*3
                Vector3[] angularB = angularBasis[i]; // 4*3
                for(int j=0; j<nBasis; j++){
                    inputs[0][j+nBasis*i] = linearB[j].x;
                    inputs[1][j+nBasis*i] = linearB[j].y;
                    inputs[2][j+nBasis*i] = linearB[j].z;
                    inputs[3][j+nBasis*i] = angularB[j].x;
                    inputs[4][j+nBasis*i] = angularB[j].y;
                    inputs[5][j+nBasis*i] = angularB[j].z;
                }
            }
        }
        {// assign values for ouputs
            outputs[0] = linearDynamic.x;
            outputs[1] = linearDynamic.y;
            outputs[2] = linearDynamic.z;
            outputs[3] = angularDynamic.x;
            outputs[4] = angularDynamic.y;
            outputs[5] = angularDynamic.z;
        }
        // Create a NN LS learning algorithm
        NonNegativeLeastSquares nnls = new NonNegativeLeastSquares()
        {
            MaxIterations = 100
        };
        // Use the algorithm to learn a multiple linear regression
        MultipleLinearRegression regression = nnls.Learn(inputs, outputs);
        // None of the regression coefficients should be negative:
        double[] coefficients = regression.Weights; // should be
        // Check the quality of the regression:
        double[] prediction = regression.Transform(inputs);
        float error = (float)new SquareLoss(expected: outputs).Loss(actual: prediction); // should be 0
        return error;
    } 
    // ===================================== for physics evaluation of contact ========================================
}
