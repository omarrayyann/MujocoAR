
# MuJoCo AR 
[![PyPI version](https://img.shields.io/pypi/v/mujoco_ar)](https://pypi.org/project/mujoco_ar/)

MuJoCo AR is a plugin for [MuJoCo](https://github.com/google-deepmind/mujoco) that enables the integration of ARKit data from a connected iOS device to control MuJoCo frames in real-time. It can also be used in non-MuJoCo applications (see the flexible setup section below). This package accompanies the MuJoCo AR iOS app that can be downloaded from [here](www.orayyan.com).

## Demos

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
    <td><img src="https://github.com/user-attachments/assets/3d496ce1-0b5d-4a1f-a6d2-dc2e19d1e3d8" width="800px" /></td>
    <td><img src="https://github.com/user-attachments/assets/8fd2b0ae-f90a-4df5-b114-3feac7c87e37" width="800px" /></td>
    <td><img src="https://github.com/user-attachments/assets/c1e927c5-a4af-4c95-a6d0-fe7f8a026c34" width="800px" /></td>
    <td><img src="https://github.com/user-attachments/assets/a58ed764-4e05-40a5-b26a-5bd896584f34" width="800px" /></td>
  </tr>
  <tr>
    <th colspan="4">
         Position and Rotation Control
      </th>
  </tr>
</table>

Examples of MuJoCo AR linked to the end-effectors of multiple manipulators can be found in this fork of [Mink](https://github.com/omarrayyann/mink-mujocoAR).

## Installation

1. iOS App: Download the iOS app from the App Store [here](www.orayyan.com)

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

You can retrieve the ARKit data including the position, rotation, button, and toggle states directly from a connected iOS device, making it flexible for usage in various applications beyond physics simulations.

```python
from mujoco_ar import MujocoARConnector

# Initialize the connector with your desired parameters
connector = MujocoARConnector()

# Start the connector
connector.start()

# Retrieve the latest AR data (after connecting the iOS device, see the guide below)
data = connector.get_latest_data()  # Returns {"position": (3, 1), "rotation": (3, 3), "button": bool, "toggle": bool}
```

## Author

Omar Rayyan (olr7742@nyu.edu)
