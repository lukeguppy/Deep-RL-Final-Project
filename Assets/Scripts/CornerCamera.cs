using UnityEngine;

public class CornerCamera : MonoBehaviour
{
    void Start()
    {
        // Get the screen dimensions
        float screenHeight = Screen.height;
        float screenWidth = Screen.width;

        // Set the camera position to the corner
        Camera.main.transform.position = new Vector3(3 * screenWidth / 4, 3 * screenHeight / 4, 0);
    }
}
