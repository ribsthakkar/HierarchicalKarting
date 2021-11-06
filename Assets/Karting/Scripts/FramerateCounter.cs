using UnityEngine;
using TMPro;

public class FramerateCounter : MonoBehaviour
{
    [Tooltip("Delay between updates of the displayed framerate value")]
    public float pollingTime = 0.05f;
    [Tooltip("The text field displaying the framerate")]
    public TextMeshProUGUI uiText;

    public KartGame.AI.KartAgent agent;

    float m_AccumulatedDeltaTime = 0f;
    int m_AccumulatedFrameCount = 0;

    void Update()
    {
        float speed = agent.m_Kart.Rigidbody.velocity.magnitude;
        float tireAge = agent.m_Kart.TireWearProportion();
        uiText.text = speed.ToString() + " \n " + tireAge.ToString();

    }
}
