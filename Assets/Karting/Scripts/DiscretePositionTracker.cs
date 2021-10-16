using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class DiscretePositionTracker : MonoBehaviour
{

    [Header("Player Colliders")]
    [Tooltip("Oval Colliders of each Player's Car")]
    public Collider Player1;
    public Collider Player2;

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

    private void OnTriggerEnter(Collider other)
    {
        if (other.Equals(Player1))
        {
            float distanceLane1 = Vector3.Distance(Player1.transform.position, Lane1.transform.position);
            float distanceLane2 = Vector3.Distance(Player1.transform.position, Lane2.transform.position);
            float distanceLane3 = Vector3.Distance(Player1.transform.position, Lane3.transform.position);
            float distanceLane4 = Vector3.Distance(Player1.transform.position, Lane4.transform.position);
            float min_distance = new float[] { distanceLane1, distanceLane2, distanceLane3, distanceLane4 }.Min();
            if (min_distance == distanceLane1)
            {
                print("Closest to Lane 1");
            } else if (min_distance == distanceLane2)
            {
                print("Closest to Lane 2");
            } else if (min_distance == distanceLane3)
            {
                print("Closest to Lane 3");
            } else if (min_distance == distanceLane4)
            {
                print("Closest to Lane 4");
            } else
            {
                print("Error in determining Lane");
            }

        }
    }
}
