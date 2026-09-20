English | [简体中文](README.zh-CN.md)

# Robot Arm

A robot control platform centered on AI-driven pick-and-place, integrating natural-language tasks, visual recognition, 3D localization, motion planning and physical execution.

[AI Pick-and-Place](#ai-driven-pick-and-place) · [Control Binding](#control-binding) · [Space](#space) · [Perception](#perception) · [Motion](#motion) · [Execution](#execution)

## AI-Driven Pick-and-Place

Robot Arm translates natural-language instructions into physical object-manipulation tasks. AI identifies grasp targets and placement regions from camera images, invokes localization and grasp planning, and follows robot execution results. Conversations, segmentation results, live video and task progress are presented in a unified browser interface.

The core workflow comprises six stages:

1. **Task interpretation:** identify the object, destination and sequence of operations.
2. **Visual recognition:** acquire camera images and identify targets through model recognition or image-region selection.
3. **3D localization:** combine corresponding depth data and calibration to locate objects and destinations.
4. **Grasp planning:** generate candidates and check reachability, environmental collisions and complete transport paths.
5. **Execution:** perform approach, gripping, transport, release and return to the working pose.
6. **Result inspection:** use task state, joint feedback and subsequent images to assess the outcome and inform further steps.

### Physical task example

> Place the purple object in the red frame, then move it from there to the center.

![Two AI-driven pick-and-place operations with conversation, segmentation and live video](.github/media/ai-pick-place.png)

User-provided screenshot of a physical operation. The left panel records recognition, region selection, manipulation and image checks. The large image on the right is the segmentation frame used for localization; the upper-right video shows the later state. The record retains the first placement's deviation outside the red frame, followed by another grasp and placement at the center as instructed.

AI coordinates existing platform functions; dedicated modules handle robot planning and execution. Model replies, software task states and physical outcomes are reported separately, distinguishing conversation completion from completed robot movement.

## Platform Capabilities

The platform supports AI pick-and-place through camera management, automatic calibration, multi-source segmentation, motion control, continuous gripping and device feedback. Physical manipulation has been completed with a StarArm-102 six-axis arm and a RealSense D415 depth camera.

**Hardware-free operation is supported.** Complete test data and virtual joint feedback cover recognition, calibration, grasp planning and simulated execution without a robot arm, depth camera or controller.

### Control interfaces

| Interface | Operation | Scope |
| --- | --- | --- |
| Natural language | Task instructions interpreted through AI tool calls | Scene inspection, state queries, movement and pick-and-place |
| Browser controls | Image inspection, target selection and explicit execution | Individual operations, configuration and manipulation without general-purpose AI |
| Gamepads and spatial controllers | Buttons, sticks and tracked movement | Continuous position, orientation and gripper control |

All interfaces share robot planning and execution capabilities.

### Five modules

| Module | Responsibility |
| --- | --- |
| Control Binding | Input-device selection, action bindings and feedback configuration |
| Space | Direction mapping, movement scale and position/orientation transforms |
| Perception | Camera management, calibration, recognition, localization and AI tasks |
| Motion | Target poses, path planning and complete manipulation tasks |
| Execution | Device connections, joint feedback, continuous gripping and motor parameters |

### Hardware-free operation and test data

With runtime dependencies and model files installed, the complete software workflow operates without external robotics hardware:

- **Simulated camera and complete test data:** a scene containing a cube, basket and ground provides matching color images, depth data and camera parameters for recognition, localization and point-cloud generation.
- **Simulated calibration:** calibration-board data generates observations from virtual arm poses, supporting sampling and calibration without a physical board.
- **Virtual joint feedback:** software execution updates joint and gripper state from commanded targets and drives the 3D robot visualization.
- **Manipulation workflow:** browser controls and simulated input support target selection, recognition, localization, planning and simulated execution. A configured general-purpose model service enables AI task orchestration.

Simulation and hardware share the business workflow, with their feedback sources explicitly identified in the interface. Simulation verifies software behavior; it does not fully model friction or slipping and does not replace physical grasp validation.

## Control Binding

Manage the relationship between buttons, sticks, tracked movement and robot actions. Device bindings remain independent of motion logic, allowing controller changes through configuration.

### Device integration

- SDL3 gamepad input and NOLO CV1 spatial tracking.
- Device names, connection state and available buttons, axes, position and orientation data.
- Independent position and orientation sources or combined input from one device.
- Browser translation and rotation controls without an external controller.
- Persistent device names and bindings.

### Action bindings

Bindings support single-button triggers, continuous axes and positive/negative button pairs. Actions such as gripper opening, closing and translation receive explicit input assignments, with direction inversion configured per binding.

Available actions include translation, arc movement, gripper rotation, movement along the gripper's pointing direction, and opening or closing. Compatible vibration devices receive gripping-load feedback.

![Button, direction and feedback-device bindings](.github/media/control-bindings.jpg)

### Input inspection

The interface displays actual button, stick and tracking input for device, binding and direction checks. Simulated input supports configuration verification without physical controllers.

![Browser direction controls, device selection and input state](.github/media/tracking.jpg)

## Space

Configure direction mappings, displacement scale and rotation behavior between input devices and the robot. These settings define how device movement affects arm position and gripper orientation.

### Direction mapping and movement scale

- Map input directions to forward/backward, left/right and up/down robot movement.
- Scale the relationship between device displacement and robot displacement.
- Establish a relative reference at control takeover and use subsequent pose changes.
- Generate continuous movement from buttons or sticks on devices without spatial tracking.
- Save spatial settings and inspect direction relationships in a 3D view.

![Spatial directions and movement-scale settings](.github/media/spatial.jpg)

### Position and orientation control

Position and orientation are processed independently. Translation with a maintained orientation and rotation at a fixed location represent distinct control modes.

Supported components include straight movement, arc movement, orientation changes at the current location, rotation around the gripper's own axis and movement along that axis. Components support independent activation and combined configuration.

The 3D view provides multiple viewing directions and current displacement and rotation values. Saved coordinate mappings remain active across subsequent operations.

![Coordinate mapping and independently selectable movement types](.github/media/spatial-mapping.jpg)

## Perception

Provide image observations, object recognition and spatial localization for AI manipulation. Camera, calibration, segmentation and AI functions share one page, with independent invocation and task-level composition.

### Camera management and acquisition

The interface presents camera sources, device information, color images and depth data. Color supplies object appearance; depth supplies distance to visible surfaces. Together they support recognition and 3D localization.

- Resolution and frame-rate selection based on reported device capabilities, with unavailable configurations disabled.
- Independent capture and downstream sampling rates, separating continuous acquisition from on-demand recognition.
- Active resolution, frame rate and capture state, distinguished from selected configuration.
- A draggable, collapsible color-video window and independently refreshed depth preview.
- Camera-driver parameter editing and persistent device configuration.

![Camera resolution, frame rate and live capture state](.github/media/camera-streams.jpg)

### Camera-to-robot calibration

Calibration establishes the camera's position and orientation relative to the robot base, converting observed object locations into the spatial reference required for movement.

The workflow uses a ChArUco black-and-white board. It moves the arm through predefined poses, collects images and combines them with actual motor angles to calculate the camera-to-robot relationship. Current pose, collected samples and results remain visible in the page.

Stages include movement, sampling, calculation and return to the working pose. Fitting error is available for review before application. Saved results remain valid across page refreshes and service restarts; recalibration has a dedicated control.

Fitting error measures sample consistency, not overall manipulation accuracy. Board dimensions, camera depth quality and mechanical accuracy also affect final positioning.

![Camera selection and the automatic calibration workflow](.github/media/perception.jpg)

### Multi-source segmentation

Segmentation extracts objects or regions from an image. Three independent, composable sources are supported:

| Source | Input and purpose |
| --- | --- |
| Prompt-based segmentation | Object descriptions or reference-image regions identify matching targets |
| Automatic segmentation | The model generates results without an object description |
| Manual segmentation | Image boxes specify objects or destinations and their names |

Results appear on the original image with model names or manual-source labels. Labels provide access to details, while the manual editor displays only manual annotations.

Each source supports independent execution and clearing. Model changes do not alter results from other sources. Manual selection provides explicit object and destination regions.

![Automatic, prompt-based and manual segmentation controls](.github/media/segmentation-controls.jpg)

![Object recognition and a placement region in the same image](.github/media/segmentation.jpg)

In this image, the grasp target originates from model recognition and the center destination from manual annotation.

### 3D localization and grasp candidates

The system calculates object position from corresponding depth data and generates grasp-pose candidates. Motion planning selects a complete solution based on reachability and subsequent transport requirements.

Recognition, localization and manipulation are triggered independently. Image recognition does not require a robot connection or extrinsic calibration; physical manipulation requires valid spatial localization and a hardware connection.

Manual pick-and-place consists of result inspection, target selection and task initiation. The page follows candidate preparation, planning, execution and the final state.

### AI tasks and model configuration

Beyond the core manipulation workflow, AI tools cover image inspection, state queries, segmentation, localization, arm movement, gripper adjustment and progress queries.

Representative instructions:

- “Look at the current image and describe the objects.”
- “Return the arm to its working pose.”
- “Move forward one centimeter along the gripper's current direction.”
- “Place the purple sponge in the center area.”

Image input supports system-camera observations and uploaded references. Conversations retain multi-turn context, session history and new-session controls, with linked tool and robot-task details.

Model names and reasoning effort support selection and manual entry. Reasoning effort requests a level of model deliberation, subject to provider support. The platform uses an OpenAI-compatible Responses interface for models with image and tool capabilities, hosted locally or remotely. Task text and selected images are sent to the configured service; credentials remain server-side.

AI replies and robot execution results are distinct. The page provides task progress, actual feedback and controls to stop the current AI and action.

## Motion

Handle target-pose solving, path planning and task execution across manual targets, continuous control and complete vision-driven manipulation.

![Robot controls with joint adjustment and a 3D pose preview](.github/media/motion.jpg)

### Target configuration and execution preview

Joint editing is separate from execution. The interface displays actual feedback and pending targets together; edited targets are submitted through the execution button.

Targets include joint angles, gripper-endpoint position and orientation, and relative movement. Relative commands distinguish robot-base directions from gripper-local directions for workspace adjustment and tool-directed approach.

Presets such as the working pose, independent gripper controls and hold-to-move/release-to-stop control are supported. The preview represents the requested pose; planning determines reachability and path validity.

The working pose is a predefined set of joint positions used at the start or end of operations.

### Complete manipulation planning

Planning selects complete executable solutions from multiple grasp candidates rather than relying on model score alone. Checks cover reachability, gripper approach, carrying and placement.

MoveIt evaluates paths against robot geometry, joint ranges and the observed environment. Collision checking during transport includes the carried object's volume.

Grasp-depth refinement uses gripper geometry and complete-path validity. Collision handling distinguishes necessary finger/target contact from environmental collisions.

![Motion modes, planning state and configuration](.github/media/motion-planning.jpg)

### Execution stages and results

Complete manipulation includes gripper preparation, approach, closing and holding, carrying through the working pose, transport to the destination, release and return. The interface reports the active stage and provides result inspection and cancellation.

Planning, execution and terminal states are identified separately. A 3D planning view supports inspection of the robot, environment and paths.

Planning success establishes an executable path, not a physically successful grasp. Physical outcomes require camera observations and gripping feedback.

## Execution

Dispatch planned actions and read joint state. Physical operation uses actual motor feedback; hardware-free operation uses virtual joint feedback from software simulation.

### Connection and actual pose

- Serial-port selection and manually entered connection paths.
- Connection state, disconnection reasons and device errors.
- Robot-pose updates from motor-reported angles.
- Comparison of actual pose, commanded targets and gripper state.
- Configurable feedback intervals and whole-arm torque enable/release.

Torque enable activates position holding; torque release removes holding effort. The interface distinguishes historical readings from live feedback and identifies interrupted connections.

![Device connection, actual robot pose and gripping settings](.github/media/arm-execution.jpg)

### Continuous gripping control

The platform reads gripper-servo load feedback and adjusts output throughout holding and transport to track the selected target. The angle at first reaching the target is not latched as a fixed holding position.

The holding target uses a 0–100 feedback scale. Target value, measured load and regulation state are displayed. Explicit gripper opening or torque release ends holding regulation.

This scale represents load feedback, not contact force in newtons. Settings depend on object material, surface friction and deformation. Load readings assist assessment but do not independently establish a stable grasp.

### Motor information and parameters

The page presents reported angles, status, voltage, current, power, raw temperature data and firmware information. Parameter descriptions cover purpose, units, write availability and application behavior.

Reading and editing operations expose progress, latest values and errors. Descriptions distinguish immediate changes from those requiring a power cycle; write confirmation does not replace application-condition checks.

![Actual motor feedback and operating data](.github/media/servo-telemetry.jpg)

![Device information, parameter descriptions and editing controls](.github/media/servo-parameters.jpg)

## Runtime and Scope

### Configuration and history

Device bindings, spatial settings, camera configuration, confirmed calibration, AI sessions and task records are persisted. Page refreshes resume state inspection without repeating actions.

Target drafts, active configuration and device feedback are presented separately. Task state reports software progress; camera images support verification of physical grasping and placement.

### Local operation and model services

With dependencies and model files available, browser controls, local recognition and manipulation do not depend on an external language-model service. Natural-language operation requires an available model service; internet requirements depend on its deployment location.

### Integrated hardware

The current integration includes a StarArm-102 six-axis arm, RA8-U35H-M servos and a RealSense D415 depth camera, with SDL3 gamepad and NOLO CV1 input support. Additional devices require adaptation and validation; support is not universal across robot and camera models.

The platform has completed physical manipulation but is not an industrial precision system. Field of view, occlusion, transparent or reflective surfaces, calibration error, mechanical play and gripper contact conditions affect results.

## Technology and Documentation

The interface uses Next.js, React and Three.js. Device and application services primarily use Rust and Dora. YOLOE provides image recognition, GraspGenX generates grasp poses, and ROS 2 with MoveIt handles motion planning. Docker manages service environments.

Implementation, device and deployment references:

- [Architecture and call chains](docs/BACKEND.md)
- [Robot integration, servo parameters and accuracy](docs/STARARM-102.md)
- [Docker and service deployment](docs/DOCKER.md)

Technical references are currently in Chinese. Screenshots show the running interface and a 3D robot driven by device feedback. Captured readings represent runtime state, not default configuration.
