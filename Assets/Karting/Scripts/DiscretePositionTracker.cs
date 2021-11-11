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
public class DiscretePositionTracker : MonoBehaviour
{
    [Header("Trigger Boxes")]
    [Tooltip("Primary Trigger Collider of Trigger Box")]
    public BoxCollider Trigger;

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

    List<float> radiuses = new List<float>();

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

    public float radiusOfLane(int initLane, int finalLane)
    {
        return (radiuses[initLane - 1] + radiuses[finalLane - 1]) / 2.0f;
    }


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
            if (!(gs <= maxGs - g_diff))
            Debug.Log("Tested gs: " + gs + " max Gs " + maxGs + " minGs " + minGs);
            return gs <= maxGs - g_diff;
        }
    }
    
    public void resetColors()
    {
        for (int i = 1; i <= 4; i++)
            getBoxColliderForLane(i).GetComponent<Renderer>().material.color = Color.magenta;
    }
}