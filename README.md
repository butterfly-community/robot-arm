English | [简体中文](README.zh-CN.md)

# Robot Arm

An intelligent control platform for physical robot arms. Connect devices, observe the workspace and control movement in a browser—or describe a task and let AI coordinate picking and placing.

[Control Binding](#control-binding) · [Space](#space) · [Perception](#perception) · [Motion](#motion) · [Execution](#execution)

![Robot controls with joint adjustment and a 3D pose preview](.github/media/motion.jpg)

## Overview

Robot Arm brings the robot, depth camera, controllers and AI into one interface. You can operate the arm step by step, or describe a task and have the system coordinate observation, object location, path planning and execution.

For example, “Place the purple sponge in the center area” can prompt AI to inspect the camera image, identify and locate the target, then start a pick-and-place task. The page shows progress, robot feedback and the result. AI uses the same functions available in the interface, so its operations can be inspected or performed directly by the user.

The project has completed physical pick-and-place with a StarArm-102 six-axis arm and a RealSense D415 depth camera. It is intended for desktop robotics, interaction research and application development. Results still depend on object material, gripper shape, camera observations and mechanical accuracy.

### Three ways to operate

| Method | How it works | Typical use |
| --- | --- | --- |
| Natural language | Describe a task; AI calls platform functions | Observe a scene, query state, move the arm or pick and place |
| Browser controls | Inspect the image, select a target and execute | Control individual steps or work without a general-purpose AI model |
| Gamepads and spatial controllers | Use buttons, sticks or tracked movement | Continuously adjust position, orientation and gripper opening |

All three methods use the same robot planning and execution capabilities. They do not require three separate robot systems.

### Five modules

| Module | Purpose |
| --- | --- |
| Control Binding | Choose which device or button controls each action |
| Space | Define how device movement translates into robot movement |
| Perception | View the camera, identify objects, locate them and issue AI tasks |
| Motion | Set target poses and plan how the robot reaches them or completes a task |
| Execution | Connect the physical arm, inspect motor feedback and adjust gripping and device settings |

## Control Binding

Assign buttons, sticks and tracked movements to robot actions. When changing controllers, bindings can be adjusted without rewriting the robot's motion logic.

### Choose input devices

- Supports gamepads through SDL3 and NOLO CV1 spatial tracking devices.
- Displays device names, connection state and available buttons, axes, position and orientation data.
- Allows one device to control position and another to control orientation, or one device to handle both.
- Provides on-screen direction controls for translation and rotation without an external controller.
- Saves device names and bindings for subsequent use.

### Assign actions

A button can trigger an action, a stick can provide continuous control, and two buttons can represent opposite directions. For example, a pair of buttons can open and close the gripper, while a stick moves the arm forward and backward. Individual bindings can be inverted when their direction does not match the preferred interaction.

Available actions include translation, movement along an arc, rotation of the gripper itself, movement along the direction it points, and gripper opening and closing. Compatible vibration devices can also receive gripping-load feedback.

![Button, direction and feedback-device bindings](.github/media/control-bindings.jpg)

### Check the input before moving

The page shows actual button, stick and tracking input. This makes it possible to check device detection and control directions before operating the arm. Simulated input can also be used to inspect bindings and direction settings when physical controllers are unavailable.

![Browser direction controls, device selection and input state](.github/media/tracking.jpg)

## Space

Define how controller movement corresponds to robot movement: which way the arm should move, how far it should travel, and whether turning a device changes the gripper's position or its orientation.

### Set directions and movement scale

- Map input-device directions to forward/backward, left/right and up/down robot movement.
- Adjust movement scale so a larger hand movement produces a smaller robot displacement, or vice versa.
- Establish a relative reference when taking control, then use changes from that reference.
- Use buttons or sticks for continuous movement when a controller has no spatial tracking.
- Save the configuration and inspect direction relationships in a 3D view.

![Spatial directions and movement-scale settings](.github/media/spatial.jpg)

### Separate position from orientation

Moving the gripper and turning it are different operations. Raising the gripper does not necessarily mean tilting it, and changing its direction does not necessarily require changing its position.

The platform supports straight movement, movement along an arc, orientation changes at the current location, rotation around the gripper's own axis and movement along that axis. These components can be enabled independently and combined for the intended interaction.

The 3D view offers several viewing directions and displays current displacement and rotation. More detailed coordinate settings remain available in the page, but do not need to be adjusted repeatedly during everyday operation.

![Coordinate mapping and independently selectable movement types](.github/media/spatial-mapping.jpg)

## Perception

Determine what is visible, where objects are and which one to manipulate. Camera setup, calibration, object selection and AI tasks share one page. They can be used independently or combined for pick-and-place.

### View and configure the camera

The page displays the camera source, device information, color images and depth data. Color images show appearance; depth data measures distance to visible surfaces, helping convert a location in the image into a position in the physical workspace.

- Select resolutions and frame rates from the device's reported capabilities; unavailable combinations remain visible but cannot be selected.
- Configure capture rate separately from the rate supplied for further processing. The camera can keep capturing while recognition uses frames only as needed.
- Inspect active resolution, frame rate and capture state, distinguishing selected settings from those currently in use.
- Observe the workspace through a draggable, collapsible color-video window and refresh the depth preview independently.
- Inspect camera-driver settings and save the device configuration.

![Camera resolution, frame rate and live capture state](.github/media/camera-streams.jpg)

### Relate the camera to the robot: calibration

The camera locates an object relative to itself, while the robot needs a position relative to its base. Calibration establishes the relationship between the two.

The platform uses a ChArUco board with a black-and-white pattern. Once the board is mounted, the robot moves through predefined poses while the camera captures images. The system combines those observations with actual motor feedback to calculate the camera's position and orientation relative to the robot. The page shows the current pose, collected samples and calculated result.

The workflow includes movement, image collection, calculation and return to the working pose. The user can inspect the fitting error before applying the result. A saved calibration remains available after a page refresh or service restart; recalibration can be started when needed.

Fitting error describes how consistently the collected samples agree. It is not a measure of every source of error in the final robot operation. Board dimensions, depth quality and mechanical accuracy also affect positioning.

![Camera selection and the automatic calibration workflow](.github/media/perception.jpg)

### Three ways to select objects

Segmentation means selecting an object or region within an image. The platform offers three methods that can be used individually or together:

| Method | How to use it |
| --- | --- |
| Prompt-based segmentation | Describe the object, or provide a region in a reference image, and let the model find a matching target |
| Automatic segmentation | Run the model without supplying an object description |
| Manual segmentation | Draw a box around an object or placement area and give it a name |

Results appear together on the original image, with labels identifying their model or manual source. Click a label to inspect details. The manual editor shows only manual annotations, keeping editing separate from model results.

Each source can be run or cleared independently. Changing the prompt-based model does not require disabling automatic segmentation or deleting manual regions. If the model does not identify a suitable destination, the user can draw one directly.

![Automatic, prompt-based and manual segmentation controls](.github/media/segmentation-controls.jpg)

![Object recognition and a placement region in the same image](.github/media/segmentation.jpg)

In this image, the grasp target was identified by a model and the center placement region was drawn manually. They are not both automatic detections.

### From an image selection to physical manipulation

After target selection, the system uses the corresponding depth image to calculate a 3D position and generates several possible grasp poses. Motion planning then searches for a pose the robot can reach while also completing the subsequent carry and placement.

Recognition, 3D localization and pick-and-place can be triggered separately. Viewing segmentation results does not require a connected robot or a completed calibration. Physical pick-and-place requires valid spatial information and a robot connection.

For manual pick-and-place, inspect the results, select the object and destination, then start the task. The page continues to report candidate preparation, planning, execution and the final result rather than stopping at request submission.

### Use natural language

AI is not limited to pick-and-place. It can inspect camera images, read robot state, invoke segmentation and localization, move the arm, adjust the gripper and query task progress.

Examples include:

- “Look at the current image and describe the objects.”
- “Return the arm to its working pose.”
- “Move forward one centimeter along the gripper's current direction.”
- “Place the purple sponge in the center area.”

AI can use images directly from the system camera or receive reference images uploaded by the user. Conversations support follow-up messages, history and new sessions. Tool operations and associated robot tasks can be expanded for inspection.

Model names and reasoning effort can be selected from lists or entered manually. Reasoning effort requests different levels of model deliberation; support depends on the model service. The platform connects through the OpenAI-compatible Responses API to model services that support images and tool use. The service may run on the local network or remotely. Task text and the images used are sent to the configured service; credentials remain on the server.

AI replies and robot execution results are displayed separately. A finished reply does not mean the robot has completed its movement. The user can inspect task progress and actual feedback, or stop the current AI and action.

### Example: AI-driven pick-and-place

Task: “Place the purple object in the red frame, then move it from there to the center.”

![Two AI-driven pick-and-place operations with conversation, segmentation and live video](.github/media/ai-pick-place.png)

A user-provided screenshot of an actual operation. The conversation records recognition, region selection, manipulation and image checks. The large image on the right is the segmentation frame used for localization; the live video above it shows the later object state. The record also acknowledges that the first placement extended outside the red frame, followed by another grasp and placement at the center as instructed.

## Motion

Turn a desired destination into robot movement. The module supports both manual targets and complete pick-and-place tasks based on visual observations.

### Preview before executing

Editing joint angles does not immediately move the robot. The page shows the actual feedback pose alongside the pending target so the effect of an edit can be inspected before execution.

In addition to joint angles, users can specify the gripper endpoint's position and orientation or move it relative to its current pose. Relative movement can follow the robot base's directions or the gripper's own directions. “Move forward” and “advance along the gripper's pointing direction” can therefore be expressed distinctly.

The platform provides presets such as the working pose, independent gripper controls and continuous hold-to-move/release-to-stop operation. The target preview shows the requested pose; planning determines whether it is reachable and whether a usable path exists.

The working pose is a predefined set of joint positions used as a starting or ending position for operations.

### Choose an executable grasp

The system considers multiple grasp poses rather than simply executing the model's highest-scoring proposal. It checks whether the arm can reach the object, whether the gripper can approach it appropriately and whether the robot can carry and place it afterward.

Path planning uses MoveIt. It checks movement using the robot's dimensions, joint ranges and the surrounding scene observed by the camera. After grasping, collision checking also includes the carried object's volume, not just the robot itself.

Grasp depth is refined using the gripper's shape and executable paths. Finger contact with the target is part of grasping; the system does not simply treat every contact as an obstacle.

![Motion modes, planning state and configuration](.github/media/motion-planning.jpg)

### Follow the complete operation

A complete task includes preparing the gripper, approaching the target, closing and holding, carrying through the working pose, moving to the destination, releasing and returning to the working pose. The page displays the current stage and supports result inspection and cancellation.

Users can distinguish completed planning from active execution and inspect the final state. A separate 3D planning view is also available for examining the robot, environment and paths.

Successful planning means the system found an executable path; it does not guarantee that an object cannot slip. Physical success still needs to be judged using camera observations and gripping feedback.

## Execution

Communicate with the physical robot, send planned movement to its motors and report what the hardware actually does.

### Connection and actual pose

- Select a device from the serial-port list or enter a connection path manually.
- Inspect the current connection, disconnection reasons and device errors.
- Display robot posture from motor-reported angles rather than merely playing a planned animation.
- Compare actual pose, commanded targets and gripper state to check whether movement has reached its destination.
- Adjust the feedback interval and enable or release torque for all motors together.

Enabling torque makes the motors hold position; releasing it removes that holding effort. Historical readings after disconnection are distinguished from current feedback so an old pose is not presented as live state.

![Device connection, actual robot pose and gripping settings](.github/media/arm-execution.jpg)

### Regulate gripping throughout the task

Gripping is not simply closing to an angle and leaving it unchanged. The platform reads load feedback from the motor driving the gripper—the servo—and continues adjusting output during holding and transport toward the selected holding target.

The target uses a 0–100 feedback scale. The page shows the target, measured load and regulation state. Explicitly opening the gripper or releasing torque ends holding regulation.

This value is not contact force measured in newtons. Soft objects, smooth objects and rigid boxes can behave differently and may require adjustment. Load changes help assess gripping, but a single reading cannot prove that an object is securely held.

### Inspect motor information and settings

The page provides reported angles, state, voltage, current, power, raw temperature data and firmware information without opening a separate vendor application. Parameter descriptions explain purpose, units, write availability and when a change takes effect.

Supported settings can be read and changed, with progress, latest values and errors displayed. Some changes apply immediately; others require a power cycle. A successful write and an effective setting are distinguished according to the parameter's documented behavior.

![Actual motor feedback and operating data](.github/media/servo-telemetry.jpg)

![Device information, parameter descriptions and editing controls](.github/media/servo-parameters.jpg)

## Everyday use and scope

### Saved settings and history

Device bindings, spatial settings, camera configuration and confirmed calibrations can be saved. AI conversations and task records are also retained so users can resume inspection after a refresh without repeating an action.

The interface distinguishes an edited target, active settings and actual device feedback. Task state describes software progress; camera images help confirm whether the object was physically picked up and placed.

### Work without an external AI service

Once dependencies and model files are available, browser control, local object recognition and pick-and-place do not depend on an external language-model service. Natural-language operation requires a working model service; internet access depends on where that service runs.

### Currently integrated hardware

The current integration uses a StarArm-102 six-axis arm, RA8-U35H-M servos and a RealSense D415 depth camera, with SDL3 gamepad and NOLO CV1 input support. Other devices require suitable adapters and validation; this is not a claim of out-of-the-box support for every robot or camera.

The project has completed physical manipulation, but it is not an industrial precision system. Camera visibility, occlusion, transparent or reflective surfaces, calibration error, mechanical play and contact between an object and the gripper can all affect results.

## Technology and further reading

The interface uses Next.js, React and Three.js. Device and application services primarily use Rust and Dora. YOLOE provides image recognition, GraspGenX generates grasp poses, and ROS 2 with MoveIt handles motion planning. Service environments are managed through Docker.

Implementation details, device parameters and deployment instructions are available in:

- [Architecture and call chains](docs/BACKEND.md)
- [Robot integration, servo parameters and accuracy](docs/STARARM-102.md)
- [Docker and service deployment](docs/DOCKER.md)

These technical documents are currently in Chinese. Screenshots show the running interface, with the 3D robot following hardware feedback. Captured values reflect the state at capture time, not default settings.
