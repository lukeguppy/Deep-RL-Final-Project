using UnityEngine;

public class ScrollZoomCamera : MonoBehaviour
{
    public float scrollSpeed = 5.0f; // Adjust this value to control the scroll speed
    public float top = 120; // Adjust this value to control the scroll speed
    public float bottom = 40; // Adjust this value to control the scroll speed

    void Update()
    {
        float scrollDelta = Input.mouseScrollDelta.y;

        if (scrollDelta != 0)
        {
            // Adjust the camera's position along the z-axis based on the scroll input
            float newZPosition = transform.position.z + scrollDelta * scrollSpeed;

            // Clamping the new position to prevent going too close or too far
            newZPosition = Mathf.Clamp(newZPosition, bottom, top); // Set minZ and maxZ as your desired limits

            // Update the camera's position
            transform.position = new Vector3(transform.position.x, transform.position.y, newZPosition);
        }
    }
}
