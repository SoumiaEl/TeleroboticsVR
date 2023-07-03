using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveRotateObject : MonoBehaviour
{
    // Variables for rotation
    public float rotationSpeed = 50.0f;
    private bool rotating = false;

    // Variables for movement
    public float moveSpeed = 0.01f;
    private Vector3 screenPoint;
    private Vector3 offset;

    void OnMouseDown()
    {
        // Translate the object's position from the world to screen point
        screenPoint = Camera.main.WorldToScreenPoint(gameObject.transform.position);

        // Get the offset between the object's position and the mouse position in screen coordinates
        offset = gameObject.transform.position - Camera.main.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, screenPoint.z));

        // Check if the mouse wheel is clicked to start rotating the object
        if (Input.GetMouseButtonDown(2))
        {
            rotating = true;
        }
    }

    void OnMouseUp()
    {
        // Stop rotating the object when the mouse button is released
        rotating = false;
    }

    void OnMouseDrag()
    {
        // Keep track of the mouse position and move the object to follow it
        Vector3 curScreenPoint = new Vector3(Input.mousePosition.x, Input.mousePosition.y, screenPoint.z);
        Vector3 curPosition = Camera.main.ScreenToWorldPoint(curScreenPoint) + offset;
        transform.position = curPosition;
    }

    void Update()
    {
        // Rotate the object as long as the mouse wheel is clicked
        if (rotating)
        {
            float rotX = Input.GetAxis("Mouse X") * rotationSpeed * Mathf.Deg2Rad;
            float rotY = Input.GetAxis("Mouse Y") * rotationSpeed * Mathf.Deg2Rad;

            transform.Rotate(Vector3.up, -rotX);
            transform.Rotate(Vector3.right, rotY);
        }
    }
}
