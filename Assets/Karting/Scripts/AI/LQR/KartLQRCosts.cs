using CenterSpace.NMath.Core;
using MathNet.Numerics.LinearAlgebra;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace KartGame.AI.LQR
{
    public abstract class KartLQRCosts : MonoBehaviour
    {
        public abstract Vector<double> getQVec();
        public abstract Matrix<double> getQMatrix();
    }

    public class LQRCheckpointReachAvoidCost: KartLQRCosts
    {
        Vector<double> targetState;
        double targetWeight;
        KartLQRDynamics currentDynamics;
        Dictionary<int, List<double>> avoidWeights;
        Dictionary<int, List<int>> avoidIndices;
        List<KartLQRDynamics> avoidDynamics;
        Matrix<double> qMat = null;
        Vector<double> qVec = null;


        public LQRCheckpointReachAvoidCost(Vector<double> targetState, double targetWeight, Dictionary<int, List<double>> avoidWeights, Dictionary<int, List<int>> avoidIndices, List<KartLQRDynamics> avoidDynamics)
        {
            this.targetState = targetState;
            this.targetWeight = targetWeight;
            this.avoidDynamics = avoidDynamics;
            this.avoidIndices = avoidIndices;
            this.avoidWeights = avoidWeights;
        }

        public override Matrix<double> getQMatrix()
        {
            if (qMat == null)
            {
                int totalDim = currentDynamics.getXDim() + avoidDynamics.Sum((d) => d.getXDim());
                qMat = CreateMatrix.Sparse<double>(totalDim, totalDim);
                int currIndex = currentDynamics.getXDim();
                foreach (int currStateIndex in avoidWeights.Keys)
                {
                    double stateIndexTotal = 0.0;
                    var idxWeights = avoidWeights[currStateIndex];
                    var idxIndices = avoidIndices[currStateIndex];
                    for (int i = 0; i < avoidDynamics.Count; i++)
                    {
                        int targetIdx = currIndex + idxIndices[i];
                        qMat[currStateIndex, targetIdx] = idxWeights[i] / 2;
                        qMat[targetIdx, currStateIndex] = idxWeights[i] / 2;
                        qMat[targetIdx, targetIdx] = -idxWeights[i];
                        stateIndexTotal -= idxWeights[i];
                    }
                    qMat[currStateIndex, currStateIndex] = stateIndexTotal + targetWeight;
                }
            }

            return qMat;
        }

        public override Vector<double> getQVec()
        {
            if (qVec == null)
            {
                int totalDim = currentDynamics.getXDim() + avoidDynamics.Sum((d) => d.getXDim());
                qVec = CreateVector.Sparse<double>(totalDim);
                qVec.SetSubVector(0, currentDynamics.getXDim(), targetState.Negate());
                qVec.Multiply(targetWeight, qVec);
            }
            return qVec;
        }
    }
}
