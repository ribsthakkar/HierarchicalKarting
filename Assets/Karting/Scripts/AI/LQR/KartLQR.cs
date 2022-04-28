using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace KartGame.AI.LQR
{
    public class KartLQR
    {
        /**
         * Solve a feedback LQ Nash game
         * We assume that the dynamics is time invariant
         * Following the equations from here: https://github.com/HJReachability/ilqgames/blob/master/derivations/feedback_lq_nash.pdf
        **/
        public static Vector<double> solveFeedbackLQR(List<KartLQRDynamics> dynamics, List<KartLQRCosts> costs, List<Vector<double>> initials, int horizon)
        {
            Matrix<double> P = null;
            Vector<double> alpha = null;
            int players = dynamics.Count;
            int totalXDim = dynamics.Sum((d) => d.getXDim());
            int totalUDim = dynamics.Sum((d) => d.getUDim());
            Dictionary<int, Tuple<int, int>> uIndices = new Dictionary<int, Tuple<int, int>>();
            int currIndex = 0;
            for (int i =0; i < players; i++)
            {
                uIndices[i] = Tuple.Create(currIndex, dynamics[i].getUDim());
                currIndex += dynamics[i].getUDim();
            }

            // Create A Matrix
            var A = CreateMatrix.Sparse<double>(0, 0);
            for(int i = 0; i < dynamics.Count;i++)
            {
                A = A.DiagonalStack(dynamics[i].getA());
            }
            List<Matrix<double>> Bs = new List<Matrix<double>>();
            
            // Create B Matrices
            for (int i = 0; i < dynamics.Count; i++)
            {
                var B = CreateMatrix.Sparse<double>(0, dynamics[i].getUDim());
                for (int j = 0; j < dynamics.Count; j++)
                {
                    if (i == j)
                        B = B.Stack(dynamics[j].getB());
                    else
                        B = B.Stack(CreateMatrix.Sparse<double>(dynamics[j].getXDim(), dynamics[i].getUDim()));
                }
                Bs.Add(B);
            }
            // Create appended initial matrix
            var initial = CreateVector.Sparse<double>(totalXDim);
            currIndex = 0;
            foreach(Vector<double> init in initials)
            {
                initial.SetSubVector(currIndex, init.Count, init);
                currIndex += init.Count;
            }

            var Zs = Enumerable.Range(0, players).Select((p) => costs[p].getQMatrix()).ToArray();
            var etas = Enumerable.Range(0, players).Select((p) => costs[p].getQVec()).ToArray();
            for (int t = horizon; t >= 0; t--)
            {
                // Construct LHS Matrix
                Matrix<double> LHS = CreateMatrix.Sparse<double>(totalUDim, 0);
                for (int i = 0; i < players; i++)
                {
                    Matrix<double> col = CreateMatrix.Sparse<double>(0, dynamics[i].getUDim());
                    for (int j = 0; j < players; j++)
                    {
                        //Debug.Log(Zs[i].RowCount + " " + Zs[i].ColumnCount);
                        //Debug.Log(Bs[i].RowCount + " " + Bs[i].ColumnCount);
                        //Debug.Log(Bs[j].RowCount + " " + Bs[j].ColumnCount);
                        if (i == j)
                        {
                            col = col.Stack(costs[i].getRMatrix() + Bs[i].TransposeThisAndMultiply(Zs[i].Multiply(Bs[i])));
                        }
                        else
                        {
                            col = col.Stack(Bs[i].TransposeThisAndMultiply(Zs[i].Multiply(Bs[j])));
                        }
                    }
                    LHS = LHS.Append(col);
                    //print(LHS.ToString());
                }
                // Construct RHS Matrix/Vector for Z/eta respectively
                var RHSMat = Bs[0].TransposeThisAndMultiply(Zs[0].Multiply(A));
                var RHSVec = CreateVector.Dense<double>(totalUDim);
                currIndex = 0;
                for (int i = 0; i < players; i ++)
                {
                    if (i > 0)
                        RHSMat = RHSMat.Stack(Bs[i].TransposeThisAndMultiply(Zs[i].Multiply(A)));
                    RHSVec.SetSubVector(currIndex, dynamics[i].getUDim(), Bs[i].TransposeThisAndMultiply(etas[i]));
                    currIndex += dynamics[i].getUDim();
                }
                // Solve for P and alpha
              
                //print("T: " + t + " " + LHS.ToString());
                //print("T: " + t + " " + RHSMat.ToString());
                //print("T: " + t + " " + RHSVec.ToString());
                P = LHS.Solve(RHSMat);
                alpha = LHS.Solve(RHSVec);
                //print("T: " + t + " B*eta" + Bs[0].TransposeThisAndMultiply(etas[0]).ToString());
                //print("T: " + t + " " + alpha.ToString());
                // Update Zs and etas
                currIndex = 0;
                var F = A - Enumerable.Range(0, players).Aggregate(CreateMatrix.Sparse<double>(totalXDim, totalXDim), (acc, k) => acc + Bs[k] * P.SubMatrix(uIndices[k].Item1, uIndices[k].Item2, 0, totalXDim));
                var beta = Enumerable.Range(0, players).Aggregate(CreateVector.Sparse<double>(totalXDim), (acc, k) => acc - Bs[k] * alpha.SubVector(uIndices[k].Item1, uIndices[k].Item2));

                for (int i = 0; i < players; i++)
                {
                    int endIndex = currIndex + dynamics[i].getUDim();
                    Zs[i] = costs[i].getQMatrix() + P.SubMatrix(uIndices[i].Item1, uIndices[i].Item2, 0, totalXDim).TransposeThisAndMultiply(costs[i].getRMatrix().Multiply(P.SubMatrix(uIndices[i].Item1, uIndices[i].Item2, 0, totalXDim))) + F.TransposeThisAndMultiply(Zs[i].Multiply(F));
                    etas[i] = costs[i].getQVec() + P.SubMatrix(uIndices[i].Item1, uIndices[i].Item2, 0, totalXDim).TransposeThisAndMultiply(costs[i].getRMatrix().Multiply(alpha.SubVector(uIndices[i].Item1, uIndices[i].Item2))) + F.TransposeThisAndMultiply(etas[i] + Zs[i].Multiply(beta));
                    currIndex = endIndex;
                }
            }
            P = P.SubMatrix(uIndices[0].Item1, uIndices[0].Item2, 0, totalXDim);
            alpha = alpha.SubVector(uIndices[0].Item1, uIndices[0].Item2);
            //Debug.Log(P.ToString());
            //Debug.Log(alpha.ToString());
            //Debug.Log(initial.ToString());
            var optimal_control = -P * initial - alpha;
            return optimal_control;
        }

    }
}
