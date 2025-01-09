# <img src="https://github.com/user-attachments/assets/9b731c7f-7ad1-4607-90aa-f6ff1830a936" width="50" align="center" alt="Logo">&nbsp;&nbsp;MuJoCo AR

[![PyPI version](https://img.shields.io/pypi/v/mujoco_ar)](https://pypi.org/project/mujoco_ar/) [![Downloads](https://static.pepy.tech/badge/mujoco_ar)](https://pepy.tech/project/mujoco_ar) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) 

MuJoCo AR is a plugin for [MuJoCo](https://github.com/google-deepmind/mujoco) that enables the integration of ARKit data from a connected iOS device to control MuJoCo frames in real-time. It can also be used in non-MuJoCo applications (see the flexible setup section below). This package accompanies the MuJoCo AR iOS app that can be downloaded from [here](https://apps.apple.com/ae/app/mujoco-ar/id6612039501). Check the [flexible setup](#flexible-setup-works-without-mujoco) if you wanna use it outside of MuJoCo. Finally, check the [FAQ](#faq) below.

### Recent Updates

- You can now trigger a vibration with configurable properties. This can be used to add haptic feedback, whether on contact with objects or to notify the user of specific events. More details in [release log](https://github.com/omarrayyann/MujocoAR/releases/tag/v1.2.0).


## Real Demo

![1127(3)](https://github.com/user-attachments/assets/9b738682-6c7c-4aad-bf5d-de39bd114780)


## MuJoCo Demos

<table>
  <tr>
    <th colspan="4">
         Position Control
      </th>
  </tr>
  <tr>
    <th colspan="2">
          <a href="https://github.com/omarrayyann/mujoco_fruit_picking" target="_blank">MuJoCo Fruits Pick and Place</a>
      </th>
      <th colspan="2">
          <a href="https://github.com/omarrayyann/mujoco_pusht" target="_blank">MuJoCo PushT</a>
      </th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/3d496ce1-0b5d-4a1f-a6d2-dc2e19d1e3d8"  /></td>
    <td><img src="https://github.com/user-attachments/assets/8fd2b0ae-f90a-4df5-b114-3feac7c87e37" /></td>
    <td><img src="https://github.com/user-attachments/assets/c1e927c5-a4af-4c95-a6d0-fe7f8a026c34"  /></td>
    <td><img src="https://github.com/user-attachments/assets/a58ed764-4e05-40a5-b26a-5bd896584f34"/></td>
  </tr>
  <tr>
    <th colspan="4">
         Position and Rotation Control
      </th>
  </tr>
   <tr>
    <th colspan="2">
          <a href="https://github.com/omarrayyann/mujoco_study_desk" target="_blank">MuJoCo Study Desk</a>
      </th>
      <th colspan="2">
          <a href="https://github.com/omarrayyann/mujoco_blocks_stacking" target="_blank">MuJoCo Blocks Stacking</a>
      </th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/e70569ce-5ade-4161-95ab-007b1d612e0a"  /></td>
    <td><img src="https://github.com/user-attachments/assets/88635d5e-63f3-41b3-af83-3af03588c84f"  /></td>
    <td><img src="https://github.com/user-attachments/assets/dbb1dbb7-5dff-4c24-88fb-9f4b8afd7d8b"  /></td>
    <td><img src="https://github.com/user-attachments/assets/df43bb40-6e58-4e94-8d1c-a4fa90359d65"  /></td>
  </tr>
</table>

Examples of MuJoCo AR linked to the end-effectors of multiple manipulators can be found in this fork of [Mink](https://github.com/omarrayyann/mink-mujocoAR). For a simpler environemnt that the ones shwon in the demo section above, check the example by using ```mjpython demo/simple.py```

## Installation

1. iOS App: Download the iOS app from the App Store [here](https://apps.apple.com/ae/app/mujoco-ar/id6612039501)

2. Python Package: Install the Python package using pip:

```bash
pip install mujoco_ar
```

## Usage

### Quick MuJoCo Setup

This setup allows you to directly control a MuJoCo frame (body, geom, or site), with the frame's position and orientation matching the ARKit data received from the connected iOS device.

```python
from mujoco_ar import MujocoARConnector

# Initialize the connector with your desired parameters
mj_ar = MujocoARConnector(mujoco_model=my_model,mujoco_data=my_data)

# Link a MuJoCo frame (link_body(), link_geom() or link_site())
mj_ar.link_body(name="eef_target")

# Start the connector
mj_ar.start()
```
### Full MuJoCo Setup

In addition to what the quick setup allows you to do, this setup allows you to automate the applying of a translation, rotation or scaling of the recieved pose. Additionally, you can pass functions to button_fn and toggle_fn to be triggered when the button or toggle are activated

```python
from mujoco_ar import MujocoARConnector

# Initialize the connector with your desired parameters
mj_ar = MujocoARConnector(
    mujoco_model=my_model, 
    mujoco_data=my_data, 
    port=8888,                                           # Optional, defaults to 8888 if not provided
    debug=False                                          # Optional, defaults to False if not provided
)

# Link a MuJoCo frame (link_body(), link_geom() or link_site())
mj_ar.link_body(
    name="eef_target",
    scale=1.0,                                           # Optional, defaults to 1.0 if not provided
    position_origin=np.array([0.0, 0.0, 0.0]),           # Optional, defaults to [0, 0, 0] if not provided
    rotation_origin=np.identity(3),                      # Optional, defaults to I(3) if not provided
    toggle_fn=my_toggle_function,                        # Optional, calls nothing if not provided
    button_fn=my_button_function,                        # Optional, calls nothing if not provided
    disable_pos=False,                                   # Optional, defaults to False if not provided
    disable_rot=False                                    # Optional, defaults to False if not provided
)

# Start the connector
mj_ar.start()
```

### Flexible Setup (works without MuJoCo):

You can retrieve the ARKit data including the position, rotation, button, and toggle states directly from a connected iOS device, making it flexible for usage in various applications beyond physics simulations. Try running ```mjpython demos/flexible_setup.py```.

```python
from mujoco_ar import MujocoARConnector

# Initialize the connector with your desired parameters
connector = MujocoARConnector()

# Start the connector
connector.start()

# Retrieve the latest AR data (after connecting the iOS device, see the guide below)
data = connector.get_latest_data()  # Returns {"position": (3, 1), "rotation": (3, 3), "button": bool, "toggle": bool}
```
## Additional Functions

```python
connector.vibrate(sharpness=0.8, intensity=0.4, duration=0.01) # Trigger a vibration on the connected device
connector.pause_updates()  # Temporarily stops receiving updates from the connected device.
connector.resume_updates() # Resumes receiving updates from the connected device.
connector.reset_position() # Resets the current position as the origin (0,0,0).

```

## FAQ

#### How can I reduce latency?

- If you're experiencing latency, try connecting your PC to your device's hotspot. This should significantly reduce latency if you're far from a router since the communication happens locally via WebSockets.

#### It is not tracking my trajectory accurately. What can I do?

- Make sure you're not covering the camera with your fingers. If the tracking is too sensitive, you can adjust the `scale` attribute when linking the frame to make it less reactive to minor movements. In my experience, the position retrived is more accurate than the rotation.

#### Where can I start?

- I highly recommend running `demo/simple.py` to test things out if you're just getting started. This demo is configured to change the sphere's color with a button press and enlarge it with a toggle, providing a basic but effective introduction to a simple configuration.
  
#### Can I use it for a non-MuJoCo application?

- Yes, check the [Flexible Setup](#flexible-setup-works-without-mujoco) out where you can retrive the pure ARKit position and rotation and use it as you wish. You wouldn't need to pass in the MuJoCo model and data in such a case.

## How to contribute?

Report any bugs you encounter to the [issue tracker](https://github.com/omarrayyann/MujocoAR/issues). Also, feel free to suggest any features. Those could be regarding the app ([iOS swift code](https://github.com/omarrayyann/MujocoAR-iOS-App)) or the python package.

## Acknowledgement

Thanks to  [@kevinzakka](https://www.github.com/kevinzakka) for the [mink](https://github.com/kevinzakka/mink) and [mjctrl](https://github.com/kevinzakka/mjctrl) libraries, which are used in the provided demos.

## Author

Omar Rayyan (olr7742@nyu.edu)
