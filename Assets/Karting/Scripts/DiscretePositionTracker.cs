using KartGame.KartSystems;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class DiscretePositionTracker : MonoBehaviour
{
    [Header("Primary Trigger Box")]
    [Tooltip("Collider of Trigger Box")]
    public BoxCollider Trigger;

    [Header("Lane Markers")]
    [Tooltip("Boxes for each Lane")]
    public BoxCollider Lane1;
    public BoxCollider Lane2;
    public BoxCollider Lane3;
    public BoxCollider Lane4;


    // Start is called before the first frame update
    void Start()
    {
        
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
            print("Closest to Lane 1");
            return 1;
        }
        else if (min_distance == distanceLane2)
        {
            print("Closest to Lane 2");
            return 2;
        }
        else if (min_distance == distanceLane3)
        {
            print("Closest to Lane 3");
            return 3;
        }
        else if (min_distance == distanceLane4)
        {
            print("Closest to Lane 4");
            return 4;
        }
        else
        {
            print("Error in determining Lane");
            return -1;
        }
    }

}