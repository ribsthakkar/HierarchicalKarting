using UnityEngine;
using TMPro;
using System.Text;

public class TelemetryViewer : MonoBehaviour
{
    [Tooltip("The text field displaying the framerate")]
    public TextMeshProUGUI uiText;


    public RacingEnvController envController;
    int[] lastCompletedLaps;
    int[] lastEpisodeSteps;
    float[] lastLapTimes;
    float[] bestLapTimes;
    float[] lastOverallTimes;
    string winner = "";

    void Start()
    {
        lastCompletedLaps = new int[envController.Agents.Length];
        lastEpisodeSteps = new int[envController.Agents.Length];
        lastLapTimes = new float[envController.Agents.Length];
        bestLapTimes = new float[envController.Agents.Length];
        lastOverallTimes = new float[envController.Agents.Length];
        int i = 0;
        foreach (KartGame.AI.KartAgent agent in envController.Agents)
        {
            lastCompletedLaps[i] = agent.m_SectionIndex / (envController.Sections.Length);
            i++;
        }
    }

    private void OnEnable()
    {
        lastCompletedLaps = new int[envController.Agents.Length];
        lastEpisodeSteps = new int[envController.Agents.Length];
        lastLapTimes = new float[envController.Agents.Length];
        bestLapTimes = new float[envController.Agents.Length];
        lastOverallTimes = new float[envController.Agents.Length];
        int i = 0;
        foreach (KartGame.AI.KartAgent agent in envController.Agents)
        {
            lastCompletedLaps[i] = agent.m_SectionIndex / (envController.Sections.Length);
            i++;
        }
    }

    void Update()
    {
        StringBuilder telemetryInfo = new StringBuilder();
        int i = 0;
        float minTimes = 1000f;
        winner = "";
        foreach(KartGame.AI.KartAgent agent in envController.Agents)
        {
            float speed = agent.m_Kart.Rigidbody.velocity.magnitude;
            float tireAge = agent.m_Kart.TireWearProportion();
            int currentLap = agent.m_SectionIndex / agent.m_envController.Sections.Length;
           if (currentLap > lastCompletedLaps[i])
            {
                lastCompletedLaps[i] = currentLap;
                lastLapTimes[i] = Time.fixedDeltaTime * (agent.m_envController.episodeSteps - lastEpisodeSteps[i]);
                if (bestLapTimes[i] < 10 || lastLapTimes[i] < bestLapTimes[i])
                {
                    bestLapTimes[i] = lastLapTimes[i];
                }
                lastEpisodeSteps[i] = agent.m_envController.episodeSteps;
            }
            else if (currentLap < lastCompletedLaps[i])
            {
                lastCompletedLaps[i] = currentLap;
                lastLapTimes[i] = 0f;
                bestLapTimes[i] = 0f;
                lastEpisodeSteps[i] = 0;
            }
            if (agent.gameObject.activeSelf)
            {
                lastOverallTimes[i] = agent.m_envController.episodeSteps * Time.fixedDeltaTime;
            }
            if (!agent.gameObject.activeSelf && lastEpisodeSteps[i] < minTimes && !winner.Equals("Tie"))
            {
                winner = agent.name;
                minTimes = lastOverallTimes[i];
            } else if (!agent.gameObject.activeSelf && lastOverallTimes[i] == minTimes)
            {
                winner = "Tie";
            }

            telemetryInfo.AppendLine(agent.name + " Speed: " + speed.ToString());
            telemetryInfo.AppendLine(agent.name + " Last Lap: " + lastLapTimes[i].ToString());
            telemetryInfo.AppendLine(agent.name + " Best Lap: " + bestLapTimes[i].ToString());
            telemetryInfo.AppendLine(agent.name + " Total Time: " + lastOverallTimes[i].ToString());
            telemetryInfo.AppendLine(agent.name + " Laps Completed: " + lastCompletedLaps[i].ToString() + "/" + agent.m_envController.laps.ToString());
            telemetryInfo.AppendLine(agent.name + " Illegal Lane Changes: " + agent.m_IllegalLaneChanges);
            telemetryInfo.AppendLine(agent.name + " Collisions: " + agent.forwardCollisions);
            telemetryInfo.AppendLine(agent.name + " Avg Target Lane Difference: " + agent.AverageLaneDifference);
            telemetryInfo.AppendLine(agent.name + " Avg Target Vel Difference: " + agent.AverageVelDifference);

            i++;
        }
        telemetryInfo.AppendLine("Winner: " + winner);

        uiText.text = telemetryInfo.ToString();

    }
}
