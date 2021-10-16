using UnityEngine;
using static KartGame.KartSystems.KartAnimation;

/// <summary>
/// This class inherits from TargetObject and represents a PickupObject.
/// </summary>
public class PickupObject : TargetObject
{
    [Header("PickupObject")]

    [Tooltip("New Gameobject (a VFX for example) to spawn when you trigger this PickupObject")]
    public GameObject spawnPrefabOnPickup;

    [Tooltip("Destroy the spawned spawnPrefabOnPickup gameobject after this delay time. Time is in seconds.")]
    public float destroySpawnPrefabDelay = 10;
    
    [Tooltip("Destroy this gameobject after collectDuration seconds")]
    public float collectDuration = 0f;

    [Tooltip("Information referring to the front left wheel of the kart.")]
    public Wheel frontLeftWheel;
    [Tooltip("Information referring to the front right wheel of the kart.")]
    public Wheel frontRightWheel;
    [Tooltip("Information referring to the rear left wheel of the kart.")]
    public Wheel rearLeftWheel;
    [Tooltip("Information referring to the rear right wheel of the kart.")]
    public Wheel rearRightWheel;

    void Start() {
        Register();
    }

    void OnCollect()
    {
        if (CollectSound)
        {
            AudioUtility.CreateSFX(CollectSound, transform.position, AudioUtility.AudioGroups.Pickup, 0f);
        }

        if (spawnPrefabOnPickup)
        {
            var vfx = Instantiate(spawnPrefabOnPickup, CollectVFXSpawnPoint.position, Quaternion.identity);
            Destroy(vfx, destroySpawnPrefabDelay);
        }
               
        Objective.OnUnregisterPickup(this);

        TimeManager.OnAdjustTime(TimeGained);

        Destroy(gameObject, collectDuration);

        if(!(frontLeftWheel is null))
        {
            WheelFrictionCurve myWfc;
            myWfc = frontLeftWheel.wheelCollider.sidewaysFriction;
            myWfc.extremumSlip /= 2;
            myWfc.asymptoteSlip /= 2;
            frontLeftWheel.wheelCollider.sidewaysFriction = myWfc;
            myWfc = frontRightWheel.wheelCollider.sidewaysFriction;
            myWfc.extremumSlip /= 2;
            myWfc.asymptoteSlip /= 2;
            frontRightWheel.wheelCollider.sidewaysFriction = myWfc;
            myWfc = rearLeftWheel.wheelCollider.sidewaysFriction;
            myWfc.extremumSlip /= 2;
            myWfc.asymptoteSlip /= 2;
            rearLeftWheel.wheelCollider.sidewaysFriction = myWfc;
            myWfc = rearRightWheel.wheelCollider.sidewaysFriction;
            myWfc.extremumSlip /= 2;
            myWfc.asymptoteSlip /= 2;
            rearRightWheel.wheelCollider.sidewaysFriction = myWfc;
        }
    }
    
    void OnTriggerEnter(Collider other)
    {
        if ((layerMask.value & 1 << other.gameObject.layer) > 0 && other.gameObject.CompareTag("Player"))
        {
            OnCollect();
        }
    }
}
