/* Class to initialise the VR and its corresponding devices (headset and controllers) */

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR; 

public class InitialiseVR : MonoBehaviour
{
    // Declaration of headset objects
    public InputDevice _rightController;
    public InputDevice _leftController;
    public InputDevice _HMD;

    // Function to initialise/update the status of the headset components to valid
    void Update()
    {
        if (!_rightController.isValid || !_leftController.isValid || !_HMD.isValid)
            InitialiseInputDevices();
    }

    /* Functionality to check if any input devices are found and initialise them */
    private void InitialiseInputDevices()
    {
        if(!_rightController.isValid)
            InitialiseInputDevice(InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.Right, ref _rightController);
        if (!_leftController.isValid) 
            InitialiseInputDevice(InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.Left, ref _leftController);
        if (!_HMD.isValid) 
            InitialiseInputDevice(InputDeviceCharacteristics.HeadMounted, ref _HMD);
    }

    private void InitialiseInputDevice(InputDeviceCharacteristics inputCharacteristics, ref InputDevice inputDevice)
    {
        // Declare a list of input devices
        List<InputDevice> devices = new List<InputDevice>();

        // Call InputDevices to see if it can find any devices with the characteristics we're looking for
        InputDevices.GetDevicesWithCharacteristics(inputCharacteristics, devices);

        // Our hands might not be active and so they will not be generated from the search.
        // We check if any devices are found here to avoid errors.
        if (devices.Count > 0)
        {
            inputDevice = devices[0];
        }
    }
}