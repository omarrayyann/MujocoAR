
# MujocoAR

A Python package that allows the retrieval of ARKit data from a connected iOS device for usage in MuJoCo or other python applications.

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

### Basic Setup (without MuJoCo cameras)

```python
from mujoco_ar import MujocoARConnector

# Initialize the connector with your desired parameters
connector = MujocoARConnector(mujoco_model=model,mujoco_data=data)

# Start the connector
connector.start()

# Get latest AR data (after connecting the iOS device, check guide below)
data = get_latest_data() # {"position": (3,1), "rotation": (3,3), "grasp": bool}
```

### Setup with a MuJoCo camera

```python
from mujoco_ar import MujocoARConnector

# Initialize the connector with your desired parameters
connector = MujocoARConnector(
    mujoco_model=my_model, 
    mujoco_data=my_data, 
    camera_name='my_camera',
    camera_frequency=10,
)

# Start the connector
connector.start()

# Get latest AR data (after connecting the iOS device, check guide below)
data = get_latest_data() # {"position": (3,1), "rotation": (3,3), "grasp": bool}
```

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Author

Omar Rayyan (olr7742@nyu.edu)
