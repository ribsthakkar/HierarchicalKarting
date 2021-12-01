using UnityEngine;
using TMPro;

public class TelemetryViewer : MonoBehaviour
{
    [Tooltip("The text field displaying the framerate")]
    public TextMeshProUGUI uiText;

    public KartGame.AI.KartAgent agent;
    public KartGame.AI.KartAgent agent2;

    float lastLapTime1 = 0f;
    float lastLapTime2 = 0f;
    float bestLapTime1 = 0f;
    float bestLapTime2 = 0f;
    int lastEpisodeSteps1 = 0;
    int lastEpisodeSteps2 = 0;
    int lastCompletedLap1;
    int lastCompletedLap2;
    float lastOverallTime1 = 0.0f;
    float lastOverallTime2 = 0.0f;

    void Start()
    {
        lastCompletedLap1 = agent.m_SectionIndex / (agent.m_envController.Sections.Length);
        lastCompletedLap2 = agent2.m_SectionIndex / (agent2.m_envController.Sections.Length);
    }

    private void OnEnable()
    {
        lastCompletedLap1 = agent.m_SectionIndex / (agent.m_envController.Sections.Length);
        lastCompletedLap2 = agent2.m_SectionIndex / (agent2.m_envController.Sections.Length);
    }

    void Update()
    {
        float speed1 = agent.m_Kart.Rigidbody.velocity.magnitude;
        float speed2 = agent2.m_Kart.Rigidbody.velocity.magnitude;
        float tireAge1 = agent.m_Kart.TireWearProportion();
        float tireAge2 = agent2.m_Kart.TireWearProportion();
        int currentLap1 = agent.m_SectionIndex / agent.m_envController.Sections.Length;
        int currentLap2 = agent2.m_SectionIndex / agent2.m_envController.Sections.Length;
        if (currentLap1 > lastCompletedLap1)
        {
            lastCompletedLap1 = currentLap1;
            lastLapTime1 = Time.fixedDeltaTime * (agent.m_envController.episodeSteps - lastEpisodeSteps1);
            if (bestLapTime1 < 10 || lastLapTime1 < bestLapTime1)
            {
                bestLapTime1 = lastLapTime1;
            }
            lastEpisodeSteps1 = agent.m_envController.episodeSteps;
        } else if (currentLap1 < lastCompletedLap1)
        {
            lastCompletedLap1 = currentLap1;
            lastLapTime1 = 0f;
            bestLapTime1 = 0f;
            lastEpisodeSteps1 = 0;
        }
        if (currentLap2 > lastCompletedLap2)
        {
            lastCompletedLap2 = currentLap2;
            lastLapTime2 = Time.fixedDeltaTime * (agent2.m_envController.episodeSteps - lastEpisodeSteps2);
            if (bestLapTime2 < 10 || lastLapTime2 < bestLapTime2)
            {
                bestLapTime2 = lastLapTime2;
            }
            lastEpisodeSteps2 = agent2.m_envController.episodeSteps;
        } else if (currentLap2 < lastCompletedLap2)
        {
            lastCompletedLap2 = currentLap2;
            lastLapTime2 = 0f;
            bestLapTime2 = 0f;
            lastEpisodeSteps2 = 0;
        }
        if (agent.gameObject.activeSelf)
            lastOverallTime1 = agent.m_envController.episodeSteps * Time.fixedDeltaTime;
        if (agent2.gameObject.activeSelf)
            lastOverallTime2 = agent2.m_envController.episodeSteps * Time.fixedDeltaTime;
        uiText.text = ""+ agent.name + "Speed: " + speed1.ToString() +
                    "\n" + agent.name + " Tire Wear: " + tireAge1.ToString() +
                    "\n" + agent.name + " Last Lap: " + lastLapTime1.ToString() +
                    "\n" + agent.name + " Best Lap: " + bestLapTime1.ToString() +
                    "\n" + agent.name + " Total Time: " + lastOverallTime1.ToString() +
                    "\n" + agent.name + " Laps Completed: " + lastCompletedLap1.ToString() + "/" + agent.m_envController.laps.ToString() +
                    "\n" + agent2.name + " Speed :" + speed2.ToString() +
                    "\n" + agent2.name + " Tire Wear :" + tireAge2.ToString() +
                    "\n" + agent2.name + " Last Lap: " + lastLapTime2.ToString() +
                    "\n" + agent2.name + " Best Lap: " + bestLapTime2.ToString() +
                    "\n" + agent2.name + " Overall Time: " + lastOverallTime2.ToString() +
                    "\n" + agent2.name + " Laps Completed: " + lastCompletedLap2.ToString() + "/" + agent2.m_envController.laps.ToString()
                    ;

    }
}
