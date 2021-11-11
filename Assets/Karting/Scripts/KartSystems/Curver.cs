using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Code pulled from StackOverflow: https://stackoverflow.com/questions/25812543/unity-make-a-curved-line-with-existing-points
/*
 * Example Usage:
 * 
 * void OnDrawGizmos(){
    Vector3[] points = Curver.MakeSmoothCurve(toVector3Array(wayPoints), 10f);
    bool ptset = false;
    Vector3 lastpt = Vector3.zero;
    for(int j = 0; j < points.Length; j++){
        Vector3 wayPoint = points[j];
        if(ptset){
            Gizmos.color = new Color(0, 0, 1, 0.5f);
            Gizmos.DrawLine(lastpt, wayPoint);
        }
        lastpt = wayPoint;
        ptset = true;
    }
    if(isCircular){
        Gizmos.DrawLine(lastpt, wayPoints[0].position);
    }

}
 * 
 */
public class Curver : MonoBehaviour
{
    //arrayToCurve is original Vector3 array, smoothness is the number of interpolations. 
    public static Vector3[] MakeSmoothCurve(Vector3[] arrayToCurve, float smoothness)
    {
        List<Vector3> points;
        List<Vector3> curvedPoints;
        int pointsLength = 0;
        int curvedLength = 0;

        if (smoothness < 1.0f) smoothness = 1.0f;

        pointsLength = arrayToCurve.Length;

        curvedLength = (pointsLength * Mathf.RoundToInt(smoothness)) - 1;
        curvedPoints = new List<Vector3>(curvedLength);

        float t = 0.0f;
        for (int pointInTimeOnCurve = 0; pointInTimeOnCurve < curvedLength + 1; pointInTimeOnCurve++)
        {
            t = Mathf.InverseLerp(0, curvedLength, pointInTimeOnCurve);

            points = new List<Vector3>(arrayToCurve);

            for (int j = pointsLength - 1; j > 0; j--)
            {
                for (int i = 0; i < j; i++)
                {
                    points[i] = (1 - t) * points[i] + t * points[i + 1];
                }
            }

            curvedPoints.Add(points[0]);
        }
        return (curvedPoints.ToArray());
    }
}