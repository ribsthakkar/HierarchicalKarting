using CenterSpace.NMath.Core;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace KartGame.AI.LQR
{
    public abstract class KartLQRCosts : MonoBehaviour
    {
        public abstract DoubleVector getQVec();
        public abstract DoubleMatrix getQMatrix();
    }

    public class LQRCheckpointReachAvoidCost: KartLQRCosts
    {
        DoubleVector targetState;
        double targetWeight;
        KartLQRDynamics currentDynamics;
        Dictionary<int, List<double>> avoidWeights;
        Dictionary<int, List<int>> avoidIndices;
        List<KartLQRDynamics> avoidDynamics;

        public LQRCheckpointReachAvoidCost(DoubleVector targetState, double targetWeight, Dictionary<int, List<double>> avoidWeights, Dictionary<int, List<int>> avoidIndices, List<KartLQRDynamics> avoidDynamics)
        {
            this.targetState = targetState;
            this.targetWeight = targetWeight;
            this.avoidDynamics = avoidDynamics;
            this.avoidIndices = avoidIndices;
            this.avoidWeights = avoidWeights;
        }

        public override DoubleMatrix getQMatrix()
        {
            int totalDim = currentDynamics.getXDim() + avoidDynamics.Sum((d) => d.getXDim());
            var output = new DoubleMatrix(totalDim, totalDim);
            int currIndex = currentDynamics.getXDim();
            foreach (int currStateIndex in avoidWeights.Keys)
            {
                double stateIndexTotal = 0.0;
                var idxWeights = avoidWeights[currStateIndex];
                var idxIndices = avoidIndices[currStateIndex];
                for(int i = 0; i < avoidDynamics.Count; i++)
                {
                    int targetIdx = currIndex + idxIndices[i];
                    output[currStateIndex, targetIdx] = idxWeights[i]/2;
                    output[targetIdx, currStateIndex] = idxWeights[i]/2;
                    output[targetIdx, targetIdx] = -idxWeights[i];
                    stateIndexTotal -= idxWeights[i];
                }
                output[currStateIndex, currStateIndex] = stateIndexTotal += targetWeight; 
            }

            return output;
        }

        public override DoubleVector getQVec()
        {
            var output = new DoubleVector(targetState);
            output *= targetWeight;

            foreach(KartLQRDynamics a in avoidDynamics)
            {
                output.Append(new DoubleVector(a.getXDim()));
            }

            return output;
        }
    }
}
