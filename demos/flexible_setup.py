from mujoco_ar import MujocoARConnector

connector = MujocoARConnector()
connector.add_position_limit([[-0.1,0.1],[-0.1,0.1],[-0.1,0.1]])
connector.start()

while True:
    data = connector.get_latest_data()
    if data["position"] is not None:
        print(data)