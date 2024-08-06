
# MujocoAR

A Python package that allows the retrieval of ARKit data from a connected iOS device for usage in MuJoCo.

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
