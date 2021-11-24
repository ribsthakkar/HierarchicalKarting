using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace KartGame.AI.LQR
{
    public class KartLQR : MonoBehaviour
    {

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
                uIndices[i] = Tuple.Create(currIndex, currIndex + dynamics[i].getUDim());
                currIndex += dynamics[i].getUDim();
            }

            // Create A Matrix
            var A = CreateMatrix.Sparse<double>(0, 0);
            for(int i = 0; i < dynamics.Count;i++)
            {
                A = A.DiagonalStack(dynamics[i].getA());
                i += 1;
            }
            List<Matrix<double>> Bs = new List<Matrix<double>>();
            
            // Create B Matrices
            for (int i = 0; i < dynamics.Count; i++)
            {
                var B = CreateMatrix.Sparse<double>(0, 0);
                for (int j = 0; j < dynamics.Count; j++)
                {
                    if (i == j)
                        B = B.Stack(dynamics[j].getB());
                    B = B.Stack(CreateMatrix.Sparse<double>(dynamics[j].getUDim(), dynamics[j].getXDim()));
                }
                Bs.Add(B);
            }

            // Create appended initial matrix
            var initial = CreateVector.Sparse<double>(totalXDim);
            currIndex = 0;
            foreach(Vector<double> init in initials)
            {
                initial.SetSubVector(currIndex, currIndex + init.Count, init);
                currIndex += init.Count;
            }

            var Zs = Enumerable.Range(0, players).Select((p) => costs[p].getQMatrix()).ToArray();
            var etas = Enumerable.Range(0, players).Select((p) => costs[p].getQVec()).ToArray();

            for (int t = horizon; t >= 0; t--)
            {
                // Construct LHS Matrix
                Matrix<double> LHS = null;
                for (int i = 0; i < players; i++)
                {
                    Matrix<double> col = null;
                    Matrix<double> offDiag = null;
                    for (int j = 0; j < players; j++)
                    {
                        if (i == j)
                        {
                            if (col != null)
                                col = col.Stack(dynamics[i].getB().TransposeThisAndMultiply(Zs[i].Multiply(dynamics[i].getB())));
                            else
                                col = dynamics[i].getB().TransposeThisAndMultiply(Zs[i].Multiply(dynamics[i].getB()));
                        }
                        else
                        {
                            //if (offDiag == null)
                            //{
                            //    offDiag = dynamics[j].getB().TransposeThisAndMultiply(Zs[j].Multiply(Enumerable.Range(0, players).Where((k) => k != j).Aggregate(,(acc, k) => acc + dynamics[k].getB())));
                            //}
                            if (col != null)
                                col = col.Stack(dynamics[i].getB().TransposeThisAndMultiply(Zs[i].Multiply(dynamics[j].getB())));
                            else
                                col = dynamics[i].getB().TransposeThisAndMultiply(Zs[i].Multiply(dynamics[j].getB()));
                        }
                    }
                    if (LHS != null)
                        LHS = LHS.Append(col);
                    else
                        LHS = col;
                }
                // Construct RHS Matrix/Vector for Z/eta respectively
                var RHSMat = dynamics[0].getB().TransposeThisAndMultiply(Zs[0].Multiply(A));
                var RHSVec = CreateVector.Dense<double>(totalUDim);
                currIndex = 0;
                for (int i = 0; i < players; i ++)
                {
                    if (i > 0)
                        RHSMat = RHSMat.Stack(dynamics[i].getB().TransposeThisAndMultiply(Zs[i].Multiply(A)));
                    RHSVec.SetSubVector(currIndex, currIndex+dynamics[i].getUDim(), etas[i]);
                    currIndex += dynamics[i].getUDim();
                }
                // Solve for P and alpha
                P = LHS.Solve(RHSMat);
                alpha = LHS.Solve(RHSVec);

                // Update Zs and etas
                currIndex = 0;
                var F = A - Enumerable.Range(0, players).Aggregate(CreateMatrix.Sparse<double>(totalXDim, totalXDim), (acc, k) => acc + dynamics[k].getB() * P.SubMatrix(uIndices[k].Item1, uIndices[k].Item2, 0, totalXDim));
                var beta = Enumerable.Range(0, players).Aggregate(CreateVector.Sparse<double>(totalXDim), (acc, k) => acc - dynamics[k].getB() * alpha.SubVector(uIndices[k].Item1, uIndices[k].Item2));

                for (int i = 0; i < players; i++)
                {
                    int endIndex = currIndex + dynamics[i].getUDim();
                                        Zs[i] = costs[i].getQMatrix() + F.TransposeThisAndMultiply(Zs[i].Multiply(F));
                    etas[i] = costs[i].getQVec() + F.TransposeThisAndMultiply(etas[i] + Zs[i].Multiply(beta));
                    currIndex = endIndex;
                }
            }
            P = P.SubMatrix(uIndices[0].Item1, uIndices[0].Item2, 0, totalXDim);
            alpha = alpha.SubVector(uIndices[0].Item1, uIndices[0].Item2);
            var optimal_control = -P * initial - alpha;
            return optimal_control;
        }

    }
}
