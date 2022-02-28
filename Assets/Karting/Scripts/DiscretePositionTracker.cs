using KartGame.KartSystems;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public enum SectionType
{
    Straight = 0,
    Turn = 1
}

/**
* Class that handles all of the information about a checkpoint
* Includes the optimal best choice of lane and calculations estimating how many Gs would be pulled when making a turn over this part of the track
**/
public class DiscretePositionTracker : MonoBehaviour
{
    [Header("Trigger Boxes")]
    [Tooltip("Primary Trigger Collider of Trigger Box")]
    public BoxCollider Trigger;
    public BoxCollider NextTrigger;

    [Header("Lane Markers")]
    [Tooltip("Boxes for each Lane")]
    public BoxCollider Lane1;
    public BoxCollider Lane2;
    public BoxCollider Lane3;
    public BoxCollider Lane4;

    // [Header("Type of Section")]
    // public SectionType sectionType;

    public float trackInsideRadius;
    public float trackLength;
    public float trackWidth;
    public bool leftTurn;
    public float turnDegrees;
    public int optimalLane;

    List<float> radiuses = new List<float>();
    [HideInInspector] public List<Vector2> finePoints = new List<Vector2>();

    void Awake()
    {
        if (isStraight())
        {
            for (int i = 0; i < 10; i++)
            {
                Vector3 interpolated = Vector3.Lerp(Trigger.transform.position, NextTrigger.transform.position, i / 10f);
                finePoints.Add(new Vector2(interpolated.x, interpolated.z));
            }
        }
        else
        {
            Vector3 center = Trigger.transform.position + (leftTurn ? -Trigger.transform.right : Trigger.transform.right) * (trackInsideRadius + trackWidth / 2);
            for (int i = 0; i < 10; i++)
            {
                Vector3 interpolated = center + Vector3.Slerp(Trigger.transform.position - center, NextTrigger.transform.position - center, i / 10f);
                finePoints.Add(new Vector2(interpolated.x, interpolated.z));
            }
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        if (leftTurn)
        {
            radiuses.Add(trackInsideRadius);
            radiuses.Add(trackInsideRadius + trackWidth * (1.0f / 4.0f));
            radiuses.Add(trackInsideRadius + trackWidth * (2.0f / 4.0f));
            radiuses.Add(trackInsideRadius + trackWidth * (3.0f / 4.0f));
        }
        else
        {
            radiuses.Add(trackInsideRadius + trackWidth * (3.0f / 4.0f));
            radiuses.Add(trackInsideRadius + trackWidth * (2.0f / 4.0f));
            radiuses.Add(trackInsideRadius + trackWidth * (1.0f / 4.0f));
            radiuses.Add(trackInsideRadius);
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    /**
    * Get box collider game object for a lane in this checkpoint
    **/
    public BoxCollider getBoxColliderForLane(int lane)
    {
        switch(lane)
        {
            case 1: return Lane1;
            case 2: return Lane2;
            case 3: return Lane3;
            case 4: return Lane4;
            default:
                print("Invalid lane number requested. Returning Trigger Box instead");
                return Trigger;
        }
    }

    /**
    * Determine which lane a player is in
    **/
    public int CalculateLane(ArcadeKart player)
    {
        float distanceLane1 = Vector3.Distance(player.transform.position, Lane1.transform.position);
        float distanceLane2 = Vector3.Distance(player.transform.position, Lane2.transform.position);
        float distanceLane3 = Vector3.Distance(player.transform.position, Lane3.transform.position);
        float distanceLane4 = Vector3.Distance(player.transform.position, Lane4.transform.position);
        float min_distance = new float[] { distanceLane1, distanceLane2, distanceLane3, distanceLane4 }.Min();
        if (min_distance == distanceLane1)
        {
          //  print("Closest to Lane 1");
            return 1;
        }
        else if (min_distance == distanceLane2)
        {
           // print("Closest to Lane 2");
            return 2;
        }
        else if (min_distance == distanceLane3)
        {
           // print("Closest to Lane 3");
            return 3;
        }
        else if (min_distance == distanceLane4)
        {
            //print("Closest to Lane 4");
            return 4;
        }
        else
        {
            //print("Error in determining Lane");
            return -1;
        }
    }

    /**
    * Calculate the average radius of turning for a given initial lane when entering checkpont and final lane when exiting checkpoint
    **/
    public float radiusOfLane(int initLane, int finalLane)
    {
        return (radiuses[initLane - 1] + radiuses[finalLane - 1]) / 2.0f;
    }

    /**
    * Estimate how much one would need to travel through the track section when entering checkpoint at initial lane and exiting at final lane
    **/
    public float distanceToTravel(int initLane, int finalLane)
    {
        if (isStraight())
        {
            float widthTraversed = (Math.Abs(initLane - finalLane) * 1.0f / 3.0f) * trackWidth;
            return Mathf.Sqrt(widthTraversed * widthTraversed + trackLength * trackLength);
        }
        else
        {
            float avgRad = radiusOfLane(initLane, finalLane);
            return (Mathf.PI / 180f) * turnDegrees * avgRad;
        }
    }

    /**
    * Compute an estimate for the overall gForce integrated when making a turn from initial to final lane
    **/
    public float tireLoad(float velocity, int initLane, int finalLane)
    {
        if (isStraight())
        {

            return distanceToTravel(initLane, finalLane) * 0.01f;
        }
        else
        {
            float gs = (velocity * velocity) / radiusOfLane(initLane, finalLane);
            return gs * distanceToTravel(initLane, finalLane) * 0.01f;
        }
    }


    public bool isStraight()
    {
        return trackInsideRadius == 0f;
        // return transform.parent.GetComponent<MeshCollider>().sharedMesh.name == "ModularTrackStraight";
    }

    /**
    * Check if a velocity is feasible to attack this section of the track given some car parameters and calculations of the track's geometry
    * Used by MCTS
    **/
    internal bool isVelFeasible(int velocity, int initLane, int finalLane, float tireWearProportion, float maxGs, float minGs)
    {
        if (isStraight())
        {
            return true;
        }
        else
        {
            float gs = ((velocity * velocity) / radiusOfLane(initLane, finalLane))/9.81f;
            float g_diff = (maxGs - minGs) * (tireWearProportion);
            //if (!(gs <= maxGs - g_diff))
            //Debug.Log("Tested gs: " + gs + " max Gs " + maxGs + " minGs " + minGs);
            return gs <= maxGs - g_diff;
        }
    }


    public void resetColors()
    {
        for (int i = 1; i <= 4; i++)
            getBoxColliderForLane(i).GetComponent<Renderer>().material.color = Color.magenta;
    }

    /**
    * Return 1 or -1 depending on whether is good to be on the left or the right of the lane. This is used in heuristics for randomly sampling lane choices
    * during RL training or for the MCTS heuristic.
    **/
    public int getOptimalLaneSign()
    {
        if (optimalLane == 1)
        {
            return 1;
        } else if (optimalLane == 4)
        {
            return -1;
        }
        return 0;
    }

    /**
    * Get optimal choice of next lane to hit from the current lane. This optimal choice is based on the best fixed racing line.
    **/
    public int getOptimalNextLane()
    {
        return optimalLane;
    }
}