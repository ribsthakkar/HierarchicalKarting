using KartGame.KartSystems;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;
using System.Collections.Generic;
using System;
using Unity.MLAgents.Actuators;
using System.Linq;
using Unity.MLAgents.Policies;

namespace KartGame.AI
{

    /// <summary>
    /// The KartAgent will drive the inputs for the KartController.
    /// </summary>
    public class EndToEndKartAgent : KartAgent
    {
        protected override void Awake()
        {
            base.Awake();
        }

        public override void CollectObservations(VectorSensor sensor)
        {
        }

    }

}
