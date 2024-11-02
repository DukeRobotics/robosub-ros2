# ROS2 Migration

We are using **JAZZY JALISCO**

## Testing

### Directory of `.mcap` and `.db3` file names

Anywhere you see `[folder_name]` in the instructions below when talking about robot recordings, replace it with:

* CV:
    * For bins: `ros2_mcap_test` (`zip` file found [here](https://dukeroboticsclub.slack.com/archives/CFCGQL55K/p1730566049146299))

### Setting Up Foxglove (do this once)

In the Docker container terminal (which should be open when you open a new terminal in VSCode, assuming you followed Vedarsh's setup instructions), run the following command to install the Foxglove bridge:

```sudo apt install ros-jazzy-foxglove-bridge```

### Setting Up `.mcap` or `.db3` files (do this once)

1. Download the `.zip` file to somewhere on your computer
2. Extract the folder somewhere
3. Copy the **entire folder** (not just the contents of the folder)
4. Paste the entire folder into the `robosub-ros2` directory
5. Ensure you did this correctly by opening a terminal and `cd` into the `robosub-ros2` directory, then `cd` into the folder you just pasted.
6. To double check that the file is formatted properly, `cd` back into the main `robosub-ros2` directory, then run ros2 bag info [folder_name]`, and ensure the output is reasonable. An example of an output is given below:
>```
>Files:             ros2_mcap_test.db3
>Bag size:          454.8 MiB
>Storage id:        sqlite3
>ROS Distro:        rosbags
>Duration:          145.600319879s
>Start:             Jun 14 2024 00:15:05.760446243 (1718324105.760446243)
>End:               Jun 14 2024 00:17:31.360766122 (1718324251.360766122)
>Messages:          4369
>Topic information: Topic: /camera/usb_camera/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 4369 | Serialization Format: cdr
>Service:           0
>Service information:
>```
### Running Nodes

1. Open another terminal
2. Run the following command:
```ros2 run [package abbreviation] [launch name]```
, e,g., `ros2 run cv depthai_spatial_detection`.
3. Repeat steps 1-2 for every node you would like to run.

### Running `.mcap` or `.db3` files

If you are using a previously-recorded data file from the robot, run the following command in a new terminal:

```ros2 bag play [folder_name] -l```

### Running Foxglove

Follow this process for when you want to test and view topics in Foxglove:

1. In a new terminal, run:
```ros2 launch foxglove_bridge foxglove_bridge_launch.xml```
2. Open **Foxglove Studio**
3. Click on the **Foxglove logo** (in the top left-hand corner, the purple hexagonal shape)
4. Click on **Open Connection**.
5. Ensure **Foxglove Websocket** is selected, and in the **WebSocket URL** field put ```ws://localhost:8765```
6. Click **Open**

### Viewing Stuff on Foxglove

First, click on **LAYOUT** (it is in small grey text, with a layout name in larger white text below, on the top right hand side).

If you have a layout file that you would like to use (ask whoever is managing your migration about if this is the case), click on **Import from file...** and select the layout.

Otherwise, you can create a new layout. Search for an appropriate panel type (again, ask whoever is managing your migration), and click on it. That should automatically add a panel of the specified type, and you can add more panels by clicking on the `Add panel` icon (it is in the top left corner, immediately to the left of the Foxglove icon, where you originally connected to `ws://localhost:8765`; it looks like a white rectangle with a + icon in the bottom right of the rectangle).

To remove a panel, click the three vertical dots in the top right hand corner and then **Remove panel**. To change the topic the panel is subscribed to, click on the settings/gear icon in the top right hand corner (immediately to the left of the three vertical dots), and under the **Panel** tab in the **General** section, you should see a dropdown titled **Topic**. Click on the dropdown and select the desired panel.

You can also easily re-arrange / re-size panels just as you might with a window on your OS.