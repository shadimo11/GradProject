using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    // The target to follow (your drone's parent object)
    public Transform target;

    // Offset from the target's position
    [SerializeField] private Vector3 offset = new Vector3(0, 3, -10);

    // How smoothly the camera moves
    [SerializeField] private float followSpeed = 5f;

    void LateUpdate()
    {
        if (target == null)
        {
            Debug.LogError("Camera target is not assigned!");
            return;
        }

        // Calculate the desired position and rotation
        Vector3 desiredPosition = target.position + target.rotation * offset;
        Quaternion desiredRotation = target.rotation;

        // Smoothly move the camera to the desired position and rotation
        transform.position = Vector3.Lerp(transform.position, desiredPosition, followSpeed * Time.deltaTime);
        transform.rotation = Quaternion.Slerp(transform.rotation, desiredRotation, followSpeed * Time.deltaTime);
    }
}