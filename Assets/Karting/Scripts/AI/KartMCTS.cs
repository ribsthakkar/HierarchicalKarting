using KartGame.AI;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Diagnostics;

public class KartMCTSNode
{
    public DiscreteGameState state;
    public KartMCTSNode parent;
    public Dictionary<DiscreteKartAction, KartMCTSNode> children;
    public float totalValue;
    public int numEpisodes;

    public KartMCTSNode(DiscreteGameState state, KartMCTSNode parent = null)
    {
        this.state = state;
        this.parent = parent;
        children = new Dictionary<DiscreteKartAction, KartMCTSNode>();
        totalValue = 0.0f;
        numEpisodes = 0;
    }
}

public class KartMCTS
{
    public static KartMCTSNode constructSearchTree(DiscreteGameState state, double T = 0.09)
    {
        KartMCTSNode root = new KartMCTSNode(state);
        var timer = new Stopwatch();
        double total = 0.0f;
        int exploredStates = 1;
        while (total < T)
        {
            timer.Reset();
            timer.Start();
            KartMCTSNode leaf = findLeaf(root);
            // UnityEngine.Debug.Log(leaf.state.lastCompletedSection);
            var simRes = simulate(leaf);
            exploredStates += simRes.Item3;
            backpropagate(simRes.Item1, simRes.Item2);
            timer.Stop();
            total += timer.Elapsed.TotalSeconds;
        }
        UnityEngine.Debug.Log("MCTS explored " + exploredStates + " states");
        return root;
    }

    public static List<DiscreteGameState> getBestStatesSequence(KartMCTSNode node)
    {
        List<DiscreteGameState> bestStates = new List<DiscreteGameState>();
        while (node.children.Count > 0)
        {
            node = node.children[upperConfidenceStrategy(node)];
            if (node.state.kartStates.All((state) => state.section == node.state.lastCompletedSection))
                bestStates.Add(node.state);
        }
        return bestStates;
    }

    private static float UCTWeight(KartMCTSNode node)
    {
        return (node.totalValue / node.numEpisodes) + Mathf.Sqrt(1.0f) * (Mathf.Log(node.parent.numEpisodes / node.numEpisodes));
    }

    public static DiscreteKartAction upperConfidenceStrategy(KartMCTSNode node)
    {
        System.Random random = new System.Random();
        int index = random.Next(node.children.Count);
        DiscreteKartAction best = node.children.Keys.ElementAt(index);
        float best_uct = UCTWeight(node.children[best]);
        foreach(var item in node.children)
        {
            float node_uct = UCTWeight(item.Value);
            if (node_uct > best_uct)
            {
                best_uct = node_uct;
                best = item.Key;
            }
        }
        return best;
    }

    private static KartMCTSNode findLeaf(KartMCTSNode root)
    {
        while (root.children.Count > 0 && root.children.Count == root.state.nextMoves().Count)
        {
            root = root.children[upperConfidenceStrategy(root)];
        }
        return root;
    }

    public static float NextGaussian()
    {
        float v1, v2, s;
        do
        {
            v1 = 2.0f * UnityEngine.Random.Range(0f, 1f) - 1.0f;
            v2 = 2.0f * UnityEngine.Random.Range(0f, 1f) - 1.0f;
            s = v1 * v1 + v2 * v2;
        } while (s >= 1.0f || s == 0f);

        s = Mathf.Sqrt((-2.0f * Mathf.Log(s)) / s);

        return v1 * s;
    }
    public static float NextGaussian(float mean, float standard_deviation)
    {
        return mean + NextGaussian() * standard_deviation;
    }
    public static float NextGaussian(float mean, float standard_deviation, float min, float max)
    {
        float x;
        do
        {
            x = NextGaussian(mean, standard_deviation);
        } while (x < min || x > max);
        return x;
    }

    private static Tuple<KartMCTSNode, List<float>, int> simulate(KartMCTSNode leaf)
    {
        System.Random random = new System.Random();
        int new_states = 0;
        while (true)
        {
            var result = leaf.state.isOver();
            // UnityEngine.Debug.Log(leaf.state.lastCompletedSection + " " + leaf.state.initialSection + " " + leaf.state.finalSection);
            if (result.Item1)
            {
                return Tuple.Create(leaf, result.Item2, new_states);
            }
            DiscreteGameState state = leaf.state;
            var nextActions = state.nextMoves().OrderBy((action) => state.envController.sectionIsStraight(state.lastCompletedSection) ? -action.lane : action.lane).ThenByDescending((action) => action.max_velocity).ToList();
            // UnityEngine.Debug.Log(nextActions[0].max_velocity);
            // int index = random.Next(nextActions.Count);
            int index = Mathf.RoundToInt(Mathf.Abs(NextGaussian(0, nextActions.Count/3, -(float)nextActions.Count + 1f, (float) nextActions.Count -1f)));
            DiscreteKartAction move = nextActions[index];
            if(!leaf.children.ContainsKey(move))
            {
                leaf.children[move] = new KartMCTSNode(state.makeMove(move), leaf);
                new_states += 1;
            }
            leaf = leaf.children[move];
        }
    }

    private static void backpropagate(KartMCTSNode node, List<float> result)
    {
        while (node != null)
        {
            node.totalValue += result[node.state.upNext()];
            node.numEpisodes += 1;
            node = node.parent;
        }
    }
}
