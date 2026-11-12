# Testing the HSV Red Pole Pipeline

This guide replays a recorded front USB camera stream, runs the red-pole HSV
detector on it, and visualizes the detector output in Foxglove. Run the ROS
commands inside the repository's Docker development container.

## 1. Build the CV package

From the repository root, build all packages once if the workspaces have not
been built yet:

```bash
source build.sh
```

For later changes to the CV package, rebuild just that package:

```bash
source build.sh cv
```

## 2. Replay the camera bag

Open a terminal in the container and, from the repository root, start the
recording in a loop. The loop is important: it keeps images arriving after the
detector starts, including when the recording reaches its end.

```bash
ros2 bag play old_competition_bags/bag_files/2026.07.16_02-52-09_PM.bag \
  --loop --topics /camera/usb/front/compressed
```

This bag contains the exact compressed-image topic consumed by the node. It is
about 29.9 GiB, so allow time for it to open. The topic filter limits playback
to the front USB images rather than replaying unrelated recorded topics.

The `old_competition_bags` directory must be mounted and visible at the path
above inside the container. In this repository's Docker Compose setup, the
host's `/mnt/d` is mounted there read-only.

## 3. Start the detector

Open a second terminal in the container and run:

```bash
ros2 run cv hsv_red_pole
```

The node subscribes to `/camera/usb/front/compressed`. It publishes the
filtered mask at `/cv/front_usb/pole_red/hsv_filtered` and the annotated
contour image at `/cv/front_usb/pole_red/all_contours`. Per-pole contour
images are published at:

- `/cv/front_usb/pole_red/pole_1/contour_image`
- `/cv/front_usb/pole_red/pole_2/contour_image`
- `/cv/front_usb/pole_red/pole_3/contour_image`

### Apply changes to the node

After changing the detector's Python logic, stop the running node with
**Ctrl+C** and start it again in the same terminal:

```bash
ros2 run cv hsv_red_pole
```

The onboard workspace uses a symlink install, so Python logic edits generally
do not need a rebuild. If you changed package metadata, entry points, or
dependencies, rebuild from the repository root in the container before
restarting the node:

```bash
source build.sh cv
ros2 run cv hsv_red_pole
```

Leave bag playback and Foxglove Bridge running while restarting the detector.
Foxglove will display the new output as soon as the restarted node publishes.

## 4. Start Foxglove Bridge

Open a third terminal in the container and run:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=28765
```

This is the command behind the repository's `fg-ws` alias. In Foxglove Studio,
connect to `ws://localhost:28765` when the container is running on your
computer. Add Image panels for the topics above to watch the mask and
detections live.

## Check that the pipeline is receiving images

In another container terminal, confirm that the bag is publishing frames:

```bash
ros2 topic hz /camera/usb/front/compressed
```

If no rate is reported, check that bag playback is still running and that the
bag path is accessible from the container. The bag player, detector, and bridge
must all be running at the same time.

## Shut down cleanly

Stop the pipeline before disconnecting the external SSD. The bag is being read
through mounts from Windows into WSL and then into Docker, so do not unplug or
eject the drive while playback or the container is still using it.

1. In the terminal running `ros2 bag play`, press **Ctrl+C** and wait for the
   player to exit.
2. In the detector and Foxglove bridge terminals, press **Ctrl+C** and wait
   for each command to exit. If `ros2 topic hz` is running, stop it with
   **Ctrl+C** as well.
3. In a WSL terminal outside the Docker container, change to the repository
   root and stop the Compose container:

   ```bash
   cd /home/ubuntu/robosub-ros2
   docker compose down
   ```

   Wait for the command to finish. This releases Docker's bind mount of the
   bag directory.
4. Only after playback has stopped and `docker compose down` has completed,
   safely eject the external SSD in Windows (for example, using **Safely
   Remove Hardware / Eject Media**). Do not unplug it until Windows confirms
   it is safe to remove.
