using CenterSpace.NMath.Core;
using MathNet.Numerics.LinearAlgebra;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace KartGame.AI.LQR
{
    /**
    * Abstract class for LQ costs that return the appropriate matrices based on the weights that would be set in the explicit class derivations
    **/
    public abstract class KartLQRCosts
    {
    
        public abstract Vector<double> getQVec();
        public abstract Matrix<double> getQMatrix();
        public abstract Matrix<double> getRMatrix();
    }

    /**
    * Areach avoid cost derivation.
    * We construct cost matrix based on a target state, and a set of weights for each of those parts of the state, a control cost, opponent target states, estimated weights for opponents to hit those target states, and the weights for avoiding opposing states
    **/
    public class LQRCheckpointReachAvoidCost: KartLQRCosts
    {
        Vector<double> targetState;
        Dictionary<int, double> targetWeights;
        double controlWeight;
        KartLQRDynamics currentDynamics;
        Dictionary<int, List<double>> avoidWeights;
        Dictionary<int, List<int>> avoidIndices;
        List<KartLQRDynamics> avoidDynamics;
        List<Vector<double>> opponentTargetStates;
        List<Dictionary<int, double>> opponentTargetWeights;
        Matrix<double> qMat = null;
        Vector<double> qVec = null;
        Matrix<double> rMat = null;


        public LQRCheckpointReachAvoidCost(Vector<double> targetState, Dictionary<int, double> targetWeights, double controlWeight, KartLQRDynamics currentDynamics, List<Vector<double>> opponentTargetStates, List<Dictionary<int, double>> opponentTargetWeights, Dictionary<int, List<double>> avoidWeights, Dictionary<int, List<int>> avoidIndices, List<KartLQRDynamics> avoidDynamics)
        {
            this.targetState = targetState;
            this.targetWeights = targetWeights;
            this.controlWeight = controlWeight;
            this.currentDynamics = currentDynamics;
            this.avoidDynamics = avoidDynamics;
            this.avoidIndices = avoidIndices;
            this.avoidWeights = avoidWeights;
            this.opponentTargetStates = opponentTargetStates;
            this.opponentTargetWeights = opponentTargetWeights;
        }

        /**
         * 2nd-order components of the quadratic costs
        **/
        public override Matrix<double> getQMatrix()
        {
            if (qMat == null)
            {
                int currIndex;
                int totalDim = currentDynamics.getXDim() + avoidDynamics.Sum((d) => d.getXDim());
                qMat = CreateMatrix.Sparse<double>(totalDim, totalDim);
                foreach (int currStateIndex in avoidWeights.Keys)
                {
                    currIndex = currentDynamics.getXDim();
                    double stateIndexTotal = 0.0;
                    var idxWeights = avoidWeights[currStateIndex];
                    var idxIndices = avoidIndices[currStateIndex];
                    for (int i = 0; i < avoidDynamics.Count; i++)
                    {
                        int targetIdx = currIndex + idxIndices[i];
                        qMat[currStateIndex, targetIdx] = idxWeights[i];
                        qMat[targetIdx, currStateIndex] = idxWeights[i];
                        qMat[targetIdx, targetIdx] = -idxWeights[i];
                        stateIndexTotal -= idxWeights[i];
                        currIndex += avoidDynamics[i].getXDim();
                    }
                    qMat[currStateIndex, currStateIndex] = stateIndexTotal;
                }
                foreach (int currStateIndex in targetWeights.Keys)
                {
                    qMat[currStateIndex, currStateIndex] += targetWeights[currStateIndex];
                }
                currIndex = currentDynamics.getXDim();
                for (int i = 0; i < opponentTargetWeights.Count; i++)
                {
                    var targetWeights = opponentTargetWeights[i];
                    foreach (int otherIndex in targetWeights.Keys)
                    {
                        qMat[currIndex + otherIndex, currIndex + otherIndex] = -targetWeights[otherIndex];
                    }
                    currIndex += avoidDynamics[i].getXDim();
                }
            }

            return qMat;
        }

        /**
         * Linear components of the quadratic cost functions
        **/
        public override Vector<double> getQVec()
        {
            if (qVec == null)
            {
                int totalDim = currentDynamics.getXDim() + avoidDynamics.Sum((d) => d.getXDim());
                qVec = CreateVector.Sparse<double>(totalDim);
                qVec.SetSubVector(0, currentDynamics.getXDim(), targetState.Negate());
                foreach (int currStateIndex in targetWeights.Keys)
                {
                    qVec[currStateIndex] = qVec[currStateIndex] * targetWeights[currStateIndex];
                }
                int currIndex = currentDynamics.getXDim();
                for (int i = 0; i < opponentTargetStates.Count; i++)
                {
                    qVec.SetSubVector(currIndex, avoidDynamics[i].getXDim(), opponentTargetStates[i]);
                    var oppTargetWeights = opponentTargetWeights[i];
                    foreach (int otherIndex in oppTargetWeights.Keys)
                    {
                        qVec[currIndex + otherIndex] = qVec[currIndex + otherIndex] * -oppTargetWeights[otherIndex];
                    }
                    currIndex += avoidDynamics[i].getXDim();
                }
            }
            return qVec;
        }

        /**
         * Basic cost matrix for the control
        **/
        public override Matrix<double> getRMatrix()
        {
            if (rMat == null)
            {
                rMat = CreateMatrix.SparseIdentity<double>(currentDynamics.getUDim()) * controlWeight;
                // rMat[1, 1] = 100;
            }
            return rMat;
        }
    }
}
