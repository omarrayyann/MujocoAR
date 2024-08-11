
# MuJoCo AR [![PyPI version](https://img.shields.io/pypi/v/mujoco_ar)](https://pypi.org/project/mujoco_ar/)

A [MuJoCo](https://github.com/google-deepmind/mujoco) plugin that allows the retrieval of ARKit data from a connected iOS device to control linked MuJoCo frames. It can be also used for non-MuJoCo applications (check the felixble setup down).

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

You can install the package using pip:

```bash
pip install mujoco_ar
```

## Usage

Here's an example of how to use MujocoAR in your Python project:

### Basic MuJoCo Setup

```python
from mujoco_ar import MujocoARConnector

# Initialize the connector with your desired parameters
mj_ar = MujocoARConnector(
    mujoco_model=my_model, 
    mujoco_data=my_data, 
    port=8888,            # Optional, defaults to 8888 if not provided
    debug=False,          # Optional, defaults to False if not provided
)


# Link a MuJoCo frame (link_body(), link_site(), link_geom())
mj_ar.link_body(
    name = "eef_target",
    scale = 1.0,                                                # Optional, defaults to 1.0 if not provided
    position_origin = np.array([0.0,0.0,0.0]),                  # Optional, defaults to [0,0,0] if not provided
    rotation_origin = np.identity(3),                           # Optional, defaults to I(3) if not provided
    toggle_fn = # function to call once the toggle is pressed   # Optional, calls nothing if not provided
    button_fn = # function to call when the button is pressed   # Optional, calls nothing if not provided
    disable_pos =  False,                                       # Optional, defaults to False if not provided
    disable_rot =  False,                                       # Optional, defaults to False if not provided
)

 link_body(self, name, scale=1.0, position_origin=np.zeros(3), rotation_origin=np.identity(3), toggle_fn=None, button_fn=None, disable_pos=False, disable_rot=False):

# Start the connector
mj_ar.start()
```

### Flexible Setup (works without MuJoCo):

```python
from mujoco_ar import MujocoARConnector

# Initialize the connector with your desired parameters
connector = MujocoARConnector()

# Start the connector
connector.start()

# Get latest AR data (after connecting the iOS device, check guide below)
data = get_latest_data() # {"position": (3,1), "rotation": (3,3), "button": bool, "toggle", bool}
```

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Author

Omar Rayyan (olr7742@nyu.edu)
