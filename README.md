
# MujocoAR

A Python package that allows the retrieval of ARKit data from a connected iOS device for usage in MuJoCo or other python applications.

## Demos

<table>
  <tr>
    <th colspan="4">
          <a href="https://github.com/omarrayyann/mujoco_fruit_picking" target="_blank">MuJoCo Fruits Pick and Place</a>
      </th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/1d74ddb7-1f43-4ce4-b994-327d4071eac5" width="500px" /></td>
    <td><img src="https://github.com/user-attachments/assets/8fd2b0ae-f90a-4df5-b114-3feac7c87e37" width="500px" /></td>
    <td><img src="https://github.com/user-attachments/assets/286428d9-93bf-456b-9ef1-322793c0bb3a" width="500px" /></td>
    <td><img src="https://github.com/user-attachments/assets/3d496ce1-0b5d-4a1f-a6d2-dc2e19d1e3d8" width="500px" /></td>
  </tr>
</table>

![ScreenRecording2024-08-07at10 20 23PM-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/91647d6e-e0af-426d-a2b1-7df453b8c8a6)

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
connector = MujocoARConnector(ar_frequency=20)

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
