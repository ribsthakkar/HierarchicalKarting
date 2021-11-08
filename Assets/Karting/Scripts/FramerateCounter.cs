using UnityEngine;
using TMPro;

public class FramerateCounter : MonoBehaviour
{
    [Tooltip("Delay between updates of the displayed framerate value")]
    public float pollingTime = 0.05f;
    [Tooltip("The text field displaying the framerate")]
    public TextMeshProUGUI uiText;

    public KartGame.AI.KartAgent agent;
    public KartGame.AI.KartAgent agent2;

    float m_AccumulatedDeltaTime = 0f;
    int m_AccumulatedFrameCount = 0;

    void Update()
    {
        float speed1 = agent.m_Kart.Rigidbody.velocity.magnitude;
        float speed2 = agent2.m_Kart.Rigidbody.velocity.magnitude;
        float tireAge1 = agent.m_Kart.TireWearProportion();
        float tireAge2 = agent2.m_Kart.TireWearProportion();
        uiText.text = "Agent1 Speed:" + speed1.ToString() + " \nAgent1 Tire Wear:" + tireAge1.ToString() + "\nAgent2 Speed:" + speed2.ToString() + " \nAgent2 Tire Wear:" + tireAge2.ToString();

    }
}
