English | [简体中文](README.zh-CN.md)

# Robot Arm

An intelligent control platform for physical robot arms, integrating multi-device input, RGB-D perception, multimodal AI, motion planning and motor feedback through a unified browser interface.

[Control Binding](#control-binding) · [Space](#space) · [Perception](#perception) · [Motion](#motion) · [Execution](#execution)

![Joint controls with actual-feedback and target-pose visualization](.github/media/motion.jpg)

## Overview

Robot Arm separates robot operation into five modules: Control Binding, Space, Perception, Motion and Execution. Operators can work directly with joints, end-effector poses and device parameters, or use natural language to invoke observation, segmentation, localization, manipulation and status queries.

AI orchestrates existing capabilities through tool calls. Dedicated modules handle 3D reconstruction, inverse kinematics, collision checking and trajectory execution. Manual operation and AI use the same business interfaces; physical devices and simulated inputs share the downstream message contracts.

The current integration supports the StarArm-102 six-axis arm, RA8-U35H-M servos and RealSense D415, with physical object pick-and-place demonstrated. The browser's 3D robot follows actual motor feedback and supports comparison between current configuration, target configuration and task state.

| Module | Main inputs | Responsibility and outputs |
| --- | --- | --- |
| Control Binding | Buttons, axes, tracked positions and device orientations | Device discovery, action mapping and input composition; unified actions and poses |
| Space | Input poses, actions and mapping configuration | Relative reference, coordinate transforms and motion semantics; robot-relative control |
| Perception | RGB-D, camera parameters, actual end-effector feedback and task instructions | Calibration, segmentation, 3D scenes, grasp candidates and AI tool orchestration |
| Motion | Relative control, joint/TCP targets, scenes and grasp candidates | Continuous control, IK, collision checking and complete-task planning; executable trajectories |
| Execution | Trajectories, gripper targets and device configuration | Serial communication, gripping regulation and telemetry; actual joints and execution results |

## Control Binding

Control Binding converts device-specific input into robot-independent actions. It manages input sources, action mappings and device feedback without requiring downstream modules to understand a controller's button layout.

### Device support and input composition

| Feature | Supported behavior |
| --- | --- |
| Gamepad input | SDL3 gamepad buttons, continuous axes and capability discovery |
| Spatial tracking | NOLO CV1 position and orientation input |
| Browser controls | Translation, pitch, turning and tool-axis rotation; hold to move and release to stop |
| Independent source selection | Position and orientation can come from different devices and form one input pose |
| Non-tracked controllers | Buttons and axes can produce relative movement without absolute tracking |
| Device management | Discovery results, connection state, reported capabilities and editable display names |
| Persistent configuration | Device names and bindings are saved without treating a temporary enumeration index as permanent identity |

### Action and feedback mapping

- Bind an action to a single button, a continuous axis or a positive/negative button pair.
- Invert the direction of a binding without changing the device's raw input.
- Configure translation, arc motion, tool rotation, tool-axis movement, gripper opening/closing and control takeover.
- Operate position changes independently from the tool's own orientation.
- Reassign physical inputs when changing controllers without modifying downstream motion logic.
- Select a vibration-capable feedback device and associate gripper load feedback with supported output capabilities.

### Input validation and diagnostics

- Inspect live button states, axis values and input poses.
- Use simulated input to examine bindings and spatial mappings.
- Expand raw absolute poses, unified control input and discovery state.
- Inspect input independently of robot motion; reading an input does not start a complete robot task.

### Binding configuration

The input adapter owns device communication and raw events. The binding layer interprets those events as actions. Button identifiers, axis identifiers and tracking capabilities remain separate from business actions; motion receives a translation, rotation or gripper intent rather than a gamepad-specific key code.

| Configuration dimension | Purpose | Typical use |
| --- | --- | --- |
| Input device | Select the source for an action | Assign position, orientation and tool operation to different devices |
| Input form | Distinguish discrete triggers from continuous values | Use buttons for actions and sticks/triggers for continuous input |
| Positive/negative directions | Define opposite operator directions | Bind forward/backward or open/close to a button pair |
| Direction inversion | Reverse an action mapping | Adjust the interaction without altering raw device values |
| Position source | Supply tracked spatial position | Produce relative displacement after takeover |
| Orientation source | Supply device orientation | Map rotation to an arc or tool-centered rotation in Space |
| Feedback device | Receive robot feedback | Express gripping changes through supported vibration output |

Action labels and direction descriptions make the meaning of a binding visible in the page. Several devices can participate in the same configuration while their individual roles remain identifiable. Combining sources does not remove the provenance of position, orientation or feedback.

### Input inspection and takeover

Input inspection observes capabilities and mapping results. Takeover establishes the reference for subsequent relative control. Verifying that a button produces an event is separate from generating a robot trajectory: actual movement still passes through the spatial transform, the selected control mode and the motion node.

Simulated input replaces the input source only. It does not introduce separate spatial conversion or execution rules. Its state is displayed so it can be distinguished from physical device acquisition.

Tracked-device diagnostics retain absolute position and orientation. Conventional controller diagnostics retain actual button and axis values. These views allow acquisition, action interpretation and transformed output to be inspected separately.

![Action forms, direction mappings and device feedback bindings](.github/media/control-bindings.jpg)

![Browser controls, device selection and input inspection](.github/media/tracking.jpg)

## Space

Space owns the relationships between tracking coordinates, operator-space semantics and robot coordinates. Coordinate conversion is performed here instead of being duplicated across input adapters, motion and execution.

### Coordinates and relative control

| Feature | Supported behavior |
| --- | --- |
| Axis mapping | Edit the mapping from input coordinates to forward, left and up control axes |
| Takeover reference | Establish a relative reference from the input position and orientation at takeover |
| Translation scale | Adjust the ratio between input displacement and robot displacement |
| Position/orientation separation | Process spatial displacement independently from device orientation |
| Input without absolute position | Integrate buttons and axes using the configured translation speed |
| Input without absolute orientation | Process relative control using the configured arc angular speed |
| Configuration management | Persist spatial mappings and restore the saved mapping while editing |

### Orientation semantics and motion components

The following components can be enabled independently:

- Base-frame translation.
- Vertical and horizontal arc motion.
- Tool-centered vertical and horizontal rotation.
- Tool-axis rotation.
- Tool-axis translation.
- Helical movement along the tool axis.

Vertical device orientation can drive a vertical arc or tool-centered pitch. Horizontal orientation can drive a horizontal arc or tool-centered turning. Changes in end-effector position and changes in tool orientation have separate meanings and can be combined according to the intended interaction.

### Spatial visualization

- Display operator space, device orientation and transformed output.
- Inspect forward/backward, left/right and up/down displacement.
- Inspect arc control values and the current control state.
- Expand configuration, transformed poses and relative-motion messages for diagnosis.

### Coordinate relationships

A tracked pose, an operator direction and a robot target are different representations. The tracking device reports its state within the tracking system. Forward, left and up describe operator-facing motion semantics. Robot movement must ultimately be interpreted in an appropriate base or tool frame.

| Concept | Meaning | Distinct from |
| --- | --- | --- |
| Absolute tracked pose | Device position and orientation within the tracking system | An absolute robot TCP target |
| Takeover reference pose | Input reference established when relative control begins | The robot's default or working pose |
| Relative translation | Displacement from the reference, transformed and scaled | Copying device position directly into robot coordinates |
| Arc control | An angular input that changes end-effector position | Rotating the tool at a fixed position |
| Tool rotation | Orientation change around tool-related axes | Opening or closing the fingers |
| Tool-axis translation | Displacement along the current tool direction | Movement along a fixed base-frame vertical axis |

The takeover reference decouples controller placement from the robot's working location. Translation scale controls magnitude, axis mapping controls direction and orientation semantics determine how a rotational input affects the robot. Each setting has a separate role.

### Configuration and output inspection

The mapping matrix is directly editable and has a restore-saved-configuration action. Component switches do not alter raw device acquisition; they determine which components contribute during transformation.

With absolute tracking, output is calculated relative to the takeover reference. With buttons or axes alone, configured speeds and input state produce relative changes. Both input types pass through the same spatial module, and Motion remains responsible for reachability and physical movement.

The spatial model and numerical views describe control-space changes. They do not certify that a motor has completed the requested movement. Actual arrival is represented by motion and execution feedback.

![Spatial state, operator-space visualization and mapping configuration](.github/media/spatial.jpg)

![Coordinate mapping, enabled components and orientation semantics](.github/media/spatial-mapping.jpg)

## Perception

Perception combines camera management, extrinsic calibration, segmentation, 3D instances and AI task orchestration. Camera/calibration settings occupy their own section, while AI, segmentation and pick-and-place are organized within a shared workspace.

### Camera acquisition and configuration

| Feature | Supported behavior |
| --- | --- |
| RGB-D acquisition | Color, depth, camera parameters and associated frame metadata |
| Depth alignment | The acquisition layer uses the camera SDK to align depth to the color-image plane |
| Dynamic stream profiles | Select device-reported resolutions, pixel formats and FPS instead of one fixed camera specification |
| Color/depth selection | Inspect and select each stream's configuration |
| Unavailable profiles | Retain unavailable options with their status instead of silently removing them |
| Separate capture/publication rates | High-rate acquisition and lower-rate perception sampling; video is not tied to perception publication |
| Driver-specific parameters | Expose RealSense controls through the shared camera interface |
| Camera lifecycle | Refresh discovery, select, save and enable, reset and disable |
| Persistent profiles | Associate profiles and applied calibration with device identity for reuse |
| Runtime monitoring | Active streams, measured rates, dropped/skipped frames, latest sequence and errors |

The color preview supports floating, dragging and collapsing. Still images have a refresh action. Depth dimensions and encoding, coordinate frame, intrinsics, distortion, projection matrix, depth scale and extrinsics are visible in the camera workspace.

Aligned RGB-D, intrinsics, timing and an extrinsic snapshot travel as associated frame data. Raw acquisition does not depend on a ROS camera driver. After a restart, the camera must be selected and enabled again, but its saved profile and calibration remain available.

### Stream configuration and runtime state

Available profiles and active profiles are distinct. Available profiles come from the driver's capability enumeration; active profiles describe the actual running stream. The page allows requested settings to be compared with the applied result rather than showing only draft form values.

| State | Purpose |
| --- | --- |
| Available color/depth profiles | Inspect reported specifications and availability |
| Active profiles | Confirm running dimensions, encoding and frame rate |
| Capture FPS | Observe the actual device acquisition rate |
| Publication FPS | Observe the rate entering the perception pipeline |
| Device frame drops | Identify frames not obtained from the acquisition side |
| Intentionally skipped frames | Account for sampling at a lower publication rate |
| Latest frame sequence | Check whether observations continue to update |
| Camera error | Inspect configuration, startup and acquisition failures |

One camera node manages high-rate capture and lower-rate perception. RGB-D frames selected for publication enter depth alignment and message construction. Color preview has separate rate handling. This does not require another camera node or running the model at video frame rate.

Vendor controls are exposed through parameter descriptions that retain driver and sensor identity. Their configuration belongs to the camera service. Segmentation, scene and motion components receive a common observation contract rather than a RealSense-specific business path.

![Camera stream profiles, publication rate and driver controls](.github/media/camera-streams.jpg)

### Image format and metric scale

The color pipeline uses RGB. Device-specific formats are decoded at the acquisition boundary. OpenCV interfaces that require BGR convert at their encoding or decoding boundary; grayscale processing interprets the input as RGB. Z16 depth is not subject to color-channel conversion. Browser display converts RGB to the RGBA representation used by the canvas.

Depth values become meters using the scale reported by the device. Back-projection uses intrinsics corresponding to the aligned image plane, followed by the extrinsic transform into the robot base frame. Changing a stream profile therefore requires the matching stream metadata rather than fixed parameters copied from another resolution.

Image pixels, depth measurements and robot coordinates are separate representations. A model's 2D box is not sent directly as a robot position. Robot target selection uses the 3D result reconstructed from the corresponding depth and calibration.

### ChArUco extrinsic calibration

- Estimate the fixed camera's transform relative to the robot base.
- Execute the device-defined working-pose, sample-pose, solve and return sequence.
- Acquire new actual motor feedback during image sampling and calculate the end-effector pose through forward kinematics.
- Use reported angles rather than commanded targets as the robot observation.
- Configure the dictionary, grid, square size, marker size and measured board dimensions.
- Display the stage, pose index, sample count, sampling wait state and solver result.
- Inspect overall translation-fit RMS, maximum residual and per-sample translation/rotation residuals.
- Persist confirmed results and retain the previous calibration values in the page.
- Recalibrate or cancel without replacing an applied calibration before confirmation.

### Calibration workflow and result semantics

The camera node owns calibration state. Motion executes the poses, and actual motor feedback provides robot observations. The frontend displays and controls the process without implementing another end-effector calculation.

| Stage | Processing | Visible feedback |
| --- | --- | --- |
| Preparation | Confirm camera, board parameters and device pose sequence | Source, board settings and pose count |
| Working pose | Request and wait for the initial working-pose movement | Motion and calibration stage |
| Pose sampling | Move, settle, capture and obtain fresh motor feedback | Pose index, wait reason and sample count |
| Extrinsic solve | Combine image observations with actual end-effector poses | Solver state, fit and residuals |
| Return | Move back to the working pose after solving | Return-movement state |
| Apply | Save the confirmed result for the associated camera | Applied parameters and recalibration entry |

Using actual feedback means sampling new motor feedback for the observation rather than substituting command targets. It does not imply hardware-trigger synchronization between exposure and serial bus reads. Image timing, feedback timing and workflow state remain distinct.

RMS summarizes consistency across the fitted samples; the maximum residual identifies the largest discrepancy. Per-sample translation and rotation residuals help inspect individual poses. These measures describe the calibration fit, not guaranteed absolute positioning accuracy throughout the robot workspace.

Board geometry, mounting and the fixed camera-to-base relationship determine whether an extrinsic calibration remains applicable. Saved results can be reused when enabling the same camera, but a changed installation requires recalibration. Intrinsic, distortion, projection and extrinsic information remain available to distinguish image formation from camera mounting geometry.

![Camera configuration, device information and saved calibration](.github/media/perception.jpg)

### Segmentation and manual annotation

| Method | Supported input |
| --- | --- |
| Prompted segmentation | YOLOE text prompts, reference images and visual prompt boxes |
| Automatic segmentation | Prompt-free object detection and segmentation |
| Manual annotation | Named target or destination boxes on the original image |

The three sources can be used individually or together, with independent run, clear and disclosure controls:

- Running one model replaces that source's result and retains the other model and manual annotations.
- Manual regions can be edited, removed and saved without rerunning a model.
- Combined results appear as boxes and titles on the original image rather than a full-image color overlay.
- Selecting a title opens source, confidence and instance details.
- Manual annotations remain explicitly manual; the manual editor does not overlay the other models' boxes.
- Segmentation uses a frozen observation, with coordinates expressed in original-image pixels.
- Loading a new frame establishes a new observation; previous 3D results and grasps are not reused as results for the new scene.

### Prompt configuration and provenance

Recognition prompts describe what the model should identify. Placement-role labels are downstream semantics; they are not silently added as another set of model input prompts. Saving configuration and running inference are separate operations.

Visual prompts support reference images and regions for targets that are difficult to specify in text. Manual annotation supplies a pixel region directly. It is not represented as model inference and is not assigned a fabricated model confidence.

| Operation | Updated data | Retained data |
| --- | --- | --- |
| Run prompted segmentation | Prompted-model results on the frozen frame | Automatic results and manual annotations |
| Run automatic segmentation | Automatic-model results on the frozen frame | Prompted results and manual annotations |
| Save manual annotations | Manual names and pixel regions | Both models' results |
| Clear one source | Results from that source | Other sources; model capability remains enabled |
| Reconstruct 3D | 3D instances from existing image regions | Frozen observation and segmentation input |
| Load a new frame | Observation and subsequent derived results | Saved camera and model configuration |

The aggregate result viewer and manual editor serve different purposes. The former combines all sources, while the latter edits manual regions only. Source identity, model name and instance details describe how a result was produced. Display scaling does not change original pixel coordinates.

![Independent automatic and prompted segmentation controls](.github/media/segmentation-controls.jpg)

![A model target and manually defined destination in one observation](.github/media/segmentation.jpg)

The grasp target in this image is a model result; the center destination is manually annotated. Task labels do not imply that the destination was classified by the model.

### 3D localization and grasp inference

- Trigger 2D segmentation and 3D reconstruction independently.
- Perform segmentation without requiring a connected robot or an extrinsic calibration.
- Reconstruct using the matching depth, intrinsics and extrinsics for the segmented frame.
- Produce object position, spatial extent and instance point clouds with observation identity.
- Reuse segmentation during reconstruction instead of rerunning the model.
- Generate gripper-pose candidates with GraspGenX.
- Provide Motion with the object, destination, candidates and observation-bound scene point cloud.
- Start manual pick-and-place through object selection, destination selection and a start action, with preparation, planning and execution progress.

### Grasp inference and scene geometry

The compute service distinguishes target and non-target environment point clouds and uses the selected robot's gripper assets to generate and filter proposals. RGB supports upstream segmentation; the current grasp model uses point clouds and gripper geometry rather than directly consuming RGB color as grasp-network input.

Environment prefiltering uses scene-point distances to reduce proposals close to surrounding geometry. Similar poses are clustered by gripper geometry displacement, retaining representative poses and their original scores. Returned candidate count and internal exploration count have different meanings. Candidate reduction does not generate an averaged replacement pose.

Perception does not replace robot-specific solving. A model score evaluates a proposal under the model's representation; it does not guarantee reachability with the current base placement, joint ranges and scene. IK, full-arm collisions and complete paths are checked by Motion.

Instance geometry represents the grasp target, while environment geometry represents surrounding obstacles. The observed target extent is used for its attachable representation. Other objects enter the planner through Octomap rather than all being converted into filled solid boxes. Observation geometry remains subject to occlusion, depth quality and segmentation accuracy.

### Multimodal AI and tool use

General models connect through an OpenAI-compatible Responses API, called by the Next.js server through AI SDK. The model orchestrates tools; robot modules retain responsibility for kinematics and trajectories.

| Tool category | Available capabilities |
| --- | --- |
| Robot state | Actual joints, TCP, gripper load, connection state and robot metadata |
| Scene state | Camera settings, current scene, segmentation and annotation sources |
| Image observation | Obtain a new RGB image or capture a frozen segmentation frame |
| Perception processing | Segment, annotate image regions, reconstruct and generate grasps |
| Manipulation | Select object/destination, start pick-and-place and follow its result |
| Arm movement | Working pose, joint targets, absolute TCP and relative TCP commands |
| Gripper operation | Opening/closing and continuous holding-target adjustment |
| Task management | Query request results and cancel a specified task |

Interaction and configuration include:

- Direct camera-image access without downloading and re-uploading a frame.
- Reference-image attachments in a conversation.
- Multi-turn context, historical session selection and new sessions.
- Persistent user messages, replies, tool records and robot-request associations.
- Configurable endpoint, model name and reasoning level.
- Selection or manual input for models and reasoning, model-catalog refresh and connection/capability checks.
- Separate draft and effective settings, with credentials retained on the server.
- Current tool, execution stage, request ID, elapsed time and final result.
- An action to stop the AI run and its current robot action.
- Independent availability of prompted, automatic and manual segmentation regardless of the general model selection.

“Inspect the current image,” “move 1 cm along the tool direction” and “put the purple sponge in the center area” use observation, relative movement and complete-task capabilities respectively. They share a tool system rather than requiring a separate script for each object.

### Conversations and robot requests

One user message may produce several tool calls or only a state query. Sessions, model runs, tool calls and robot requests have distinct records linked in the interface.

| Record | Main contents | Purpose |
| --- | --- | --- |
| Session | User messages, model replies and accumulated context | Continue a task or return to earlier discussions |
| Current run | Model, reasoning settings and runtime state | Inspect processing for this turn |
| Tool call | Name, input, result and error | Identify which platform capabilities were invoked |
| Robot request | Request ID, stage and action result | Follow a physical or perception operation |
| Image reference | Camera observation or supplied reference image | Associate model interpretation with its input |

A completed model reply and a completed robot action are distinct events. Request acceptance is also separate from physical grasp confirmation. Tools can query the original request, and the model can use the returned result while the operator inspects the same operation in the manual interface.

General-model settings and local perception settings are independent. Changing the Responses model, reasoning level or endpoint does not rewrite YOLOE labels or disable manual annotation. A model endpoint with suitable vision and tool-use capabilities can provide orchestration while the local robot interface remains unchanged.

Task text and images used by the model are sent to the configured endpoint. Credentials remain server-side. The endpoint may be local or remote; the platform does not implement subscription-account login or third-party account conversion.

![Shared AI, segmentation and manipulation workspace](.github/media/ai-workspace.jpg)

The task text is an unsent example. Existing model results and manual annotations are shown on the right.

## Motion

Motion uses ROS 2, MoveIt 2, Servo and MoveIt Task Constructor for continuous control, discrete movement and manipulation tasks.

### Target control and pose preview

| Feature | Supported behavior |
| --- | --- |
| Joint targets | Edit angles, then explicitly execute |
| Draft targets | Slider changes do not immediately drive the arm; restore the actual pose to discard edits |
| Dual-pose visualization | Show actual feedback and the unexecuted target together |
| Named targets | Use device-defined configurations such as the working pose |
| Independent gripper control | Execute a tool target without specifying all arm joints |
| Absolute TCP | Base-frame position and orientation targets |
| Relative TCP | Translation and rotation in the base or tool frame |
| Current/target information | Compare position, quaternion orientation and joint state |
| Per-request settings | Override settings such as speed for an ordinary motion request |

TCP denotes the tool center point. Relative targets are resolved using actual feedback at the start of the task. The browser model provides geometric preview; planning determines reachability and collision validity.

### Target representations

Joint targets specify robot configuration. TCP targets specify the end-effector in space and require kinematic solving to obtain joint movement. Both rely on the same device model but describe different quantities.

| Representation | Reference | Use |
| --- | --- | --- |
| Joint angles | Device business-joint definitions | Explicit robot configurations |
| Named target | Device-defined joint configuration | Repeated use of a working or other preset pose |
| Absolute TCP | Robot base frame | A specified spatial position and orientation |
| Base-relative TCP | Current feedback and base axes | Movement along fixed spatial directions |
| Tool-relative TCP | Current feedback and tool axes | Approach along the tool direction or rotate the tool |
| Gripper target | Tool actuator | Opening/closing without an accompanying six-axis target |

Positions are metric and orientations use quaternions. Joint angles are presented in degrees for the operator and represented in the corresponding radians by the robot model and planner. Signs, zero references and TCP definitions belong to the device integration rather than independent page-specific offsets.

A preview may describe a valid geometric target that is not reachable. If planning fails, actual feedback is not replaced with the draft. Restoring the current pose establishes a new editing reference from actual state. Targets, plans and measured state remain distinguishable.

### Continuous control and complete solutions

- MoveIt Servo processes continuous relative control.
- Discrete joint and TCP targets use motion planning and controllers.
- Multiple grasp candidates participate in IK and complete-task solving.
- Failed candidate IK does not enter execution; execution uses a complete planned solution.
- Candidate quality, posture changes and motion cost contribute to ranking.
- Gripper geometry and collision checks refine approach and engagement depth.
- Relative control, manual targets and perception tasks share models, frames and execution feedback.

### Planning resources and task scheduling

Robot models, planning plugins and planner resources are initialized and reused by the motion service. Individual tasks construct their MTC stages and update the bound scene instead of starting another ROS environment or reloading all resources.

A single motion node coordinates Servo and discrete tasks. Discrete task execution does not compete with another concurrently transmitted relative-control path.

Complete-solution search identifies a candidate capable of finishing the task. Once an initial complete solution is available, depth and approach-plane variants may be refined for that candidate using the same start state and map. Actual execution uses one selected complete solution; it does not capture another frame after approach and silently select a different target.

This refinement belongs to one task's planning process. It is distinct from execution-time re-observation and replanning. Several solving stages do not imply several physical attempts, and solution ranking does not claim an exhaustive global optimum across every candidate.

### Grasp orientation and engagement depth

A candidate provides gripper position, orientation and model score. The motion layer also considers fingertip-line tilt, target span along the closing direction, the target's relation to the gripper and joint movement cost.

Fingertip-line tilt is not the orientation of the entire finger-opening plane. This criterion does not fix the gripper body parallel to the ground.

Depth refinement searches geometric variants that support a complete executable path. It does not cap approach distance solely using the object's vertical size. A deeper result can replace the original only when approach, gripping and subsequent transport remain part of a complete solution.

Model environment filtering, candidate end-effector collision checking and full-arm path checking address different levels of the problem. The first reduces invalid proposals; the latter checks robot-specific executability. None independently certifies physical grasp success.

![Control modes, planning status and per-request configuration](.github/media/motion-planning.jpg)

### Environment collisions and manipulation stages

- Build a task environment snapshot from observation point clouds using MoveIt's official Octomap component.
- Bind the target, candidates and observation within the same manipulation request.
- Manage target contact, grasping, attachment, carrying and release through MTC stages.
- Include the attached object's volume in transport collision checking.
- Organize approach, closing, carrying, release and return to the working pose.
- Select placement posture with continuity to transport and subsequent movement in consideration.

### Task stages and attachment

| Stage | Robot operation | Scene handling |
| --- | --- | --- |
| Preparation | Prepare the gripper opening for the candidate | Establish the observation-bound task map |
| Approach | Move along the candidate approach | Check gripper and arm paths and stage-specific target contact |
| Closing | Begin holding-feedback regulation | Attach the target to the tool |
| Carry via working pose | Move with the object through the working pose | Check collisions with the carried volume |
| Transport/release | Reach the placement area and open | Detach and process release |
| Return | Return to the working pose and complete tool cleanup | Clean up task-specific scene state |

The target is managed separately from other environment voxels. Necessary finger/target contact is handled at the appropriate stage. The same target must not remain both an obstacle at its former location and an attached body throughout transport.

The environment remains the current task's observation snapshot. Picking up the target does not require continually republishing an old point cloud.

Leaving an existing support contact has different semantics from free motion. Stage handling permits the relevant existing support relationship without deepening it, then restores ordinary collision checking after separation. This is not a global switch disabling environmental collisions.

The release target is associated with the attached object's spatial position. Object center, object-bottom height and TCP height are different quantities and should not be substituted for one another.

### State, queries and visualization

- Inspect request identity, stage, complete solutions, execution state and result code.
- View trajectory point count and planned duration.
- Query results by request ID and resume inspection after refreshing.
- Request cancellation and continue following the actual terminal state.
- Expand MoveIt/Servo state, raw request results and robot metadata.
- Open browser-accessible RViz to inspect robot geometry, point clouds and the planning scene.

## Execution

The native Rust execution node owns FashionStar serial communication, motor control, actual feedback and continuous gripper regulation.

### Connections and measured state

| Feature | Supported behavior |
| --- | --- |
| Serial selection | Enumerated ports and manually entered paths |
| Connection state | Current endpoint, connection/disconnection and errors |
| Actual joint feedback | Compute robot configuration from reported motor angles |
| 3D feedback | Show the actual model, last command and tool target |
| Command/feedback comparison | Distinguish requested targets from measured arrival |
| Feedback interval | Configure the polling period |
| Whole-arm torque control | Enable or release torque across all motors |
| Historical-state labeling | Do not present cached telemetry as current after disconnection |

### Continuous gripping regulation

- Adjust gripping from servo load feedback through holding and transport.
- Keep the holding objective separate from the gripper's position target.
- Exit regulation on explicit opening or torque release.
- Configure a normalized holding-feedback target from 0 to 100.
- Inspect the target, measured load, control power, sample time and regulation state.
- Treat the scale as load feedback rather than force in newtons; use camera observation when judging physical engagement.

### Model feedback and tool state

The solid robot represents actual feedback; the translucent robot represents the last command. Tool, grasp and placement targets have separate visual markers. The view supports comparison between requested and physical state without inferring hardware position from a planned animation.

Device metadata provides joints, tool actuators, model assets and frame relationships. The interface associates records by joint and actuator keys instead of deriving mechanical meaning from list order. Joint labels and current device parameter annotations can be enabled when needed.

Actual feedback preserves device-reported values. State processing used for planning does not replace the raw motor observation needed by calibration. The TCP, structural end link and moving fingers are separately defined to maintain a consistent tool reference across display, calibration and grasping.

![Execution connection, measured robot state and holding target](.github/media/arm-execution.jpg)

### Target, feedback and control output

| Quantity | Meaning | Purpose |
| --- | --- | --- |
| Gripper angle | Tool actuator position | Describe opening/closing geometry |
| Holding-feedback target | Normalized desired load | Set the holding-regulation objective |
| Measured load | A value derived from new servo monitoring data | Observe current drive load |
| Control power | The dynamically regulated output limit | Adjust with feedback rather than remain fixed |
| Feedback timestamp | Time associated with the sample | Distinguish new data from historical readings |
| Regulation state | Whether continuous gripping control is active | Relate feedback to gripper intent and transport |

Reaching the target load does not latch the first corresponding angle as a final holding position. Regulation continues with subsequent samples. Changing the holding target affects later control, but angle and electrical power are not interpreted as constant physical contact force.

Friction, material deformation, inertia and sampling intervals affect load. Load changes can inform grasp analysis, while camera observations and action outcomes remain available. A nonzero reading alone is not treated as confirmation of a held object.

### Servo telemetry and parameter management

- Read reported motor angles, turn counts and status flags.
- Inspect voltage, current, power and raw temperature data.
- Read firmware codes, device information and supported configuration.
- Write supported parameters with units, descriptions, write availability and application behavior.
- Inspect read progress, latest read times and per-field errors.
- Schedule queries, replies and writes on one serial channel rather than competing connections.

### Serial transactions and parameter application

The driver serializes transactions on a physical port. After a query is sent, it waits for the corresponding reply. During that interval, the latest pending complete motion target is retained and sent when the transaction permits.

Asynchronous node scheduling does not mean interleaving several independent request/reply exchanges on the bus. Parameter scans and live feedback share the scheduler, exposing progress without creating another serial connection.

Public parameters follow write/readback verification. Internal parameters follow the relevant vendor write procedure. Parameter help distinguishes readability, immediate effect and settings that require a power cycle. Readback consistency verifies the written value; it does not override the device's application requirements.

Disconnection clears pending targets. Reconnection does not replay a backlog of old movement commands. Whole-arm torque enable holds the current feedback positions rather than jumping to an earlier target. An unavailable selected physical endpoint is reported as a connection error, not replaced with simulated execution results.

![Live servo telemetry and raw device state](.github/media/servo-telemetry.jpg)

![Servo information, parameter help and editing controls](.github/media/servo-parameters.jpg)

## Shared data flow and persistence

- CameraFrameBundle associates RGB-D, intrinsics, timing and extrinsic snapshots.
- WorldScene associates instances, candidates and binary point clouds without expanding large XYZ payloads into browser JSON.
- Spatial transformation, scene orchestration, planning and execution have explicit owners; the frontend does not duplicate robot algorithms.
- Device settings, spatial mappings, camera profiles and confirmed calibrations are persisted by their owning services.
- Redis stores request and conversation metadata; conversation images use separate storage.
- The UI, AI and execution feedback associate acceptance, planning, execution and terminal states through request identity.
- Software completion, gripping feedback and visual confirmation remain separate observations.

### Request lifecycle

| Level | What it establishes | What it does not establish |
| --- | --- | --- |
| Registration/acceptance | A request was received and assigned its identity | Planning has succeeded |
| Preparation/planning | Candidates or task solutions are being produced | Motors have begun moving |
| Execution | A selected solution is being run by the controller | The object is securely held |
| Software terminal state | Completion, failure or cancellation was returned | The final physical outcome was visually confirmed |
| Images/telemetry | Camera, actual joints and measured load are available | One signal describes every aspect of the physical state |

The page follows the same request ID throughout the operation. An old result from another task does not end the current wait. Requests support independent lookup; refreshing resumes observation rather than resubmitting the action.

Records unfinished at a service restart retain uncertainty. They are not automatically classified as successful or replayed. Request persistence supports inspection and correlation, not a claim of exactly-once physical execution.

Cancellation acceptance and the eventual cancelled result are also distinct. The system requests cancellation of the corresponding task and waits for actual controller feedback. Cancelling does not implicitly open the gripper or move to another pose. Scene state associated with a held object must not be assumed empty before release.

### Ownership and extension

Input adapters own device-specific input behavior. The camera service owns its SDK, capture and calibration. The compute service owns model weights and gripper inference assets. Robot motion owns kinematics and ROS dependencies, while execution owns the serial protocol.

The frontend edits configuration, invokes operations and displays results. It does not maintain a second kinematic model for solving robot actions.

Large images and point clouds are handled separately from compact status messages. The gateway provides state updates and on-demand resources; request history stores results and references rather than acting as an image bus.

Video and perception sampling have separate rate handling. Increasing the desired observation rate in the browser does not require running every model at the same frequency.

Simulation supplies test input through the formal contracts and uses the same transformations, robot metadata and downstream motion/execution flow. Supporting a new device requires an adapter and validation of its reported capabilities; generic functions do not gain a parallel business pipeline for each hardware model.

## Technology and current hardware

| Layer | Components |
| --- | --- |
| Operator interface | Next.js, React, Three.js |
| AI orchestration | AI SDK, OpenAI-compatible Responses API |
| Nodes and messages | Rust, Dora, Arrow |
| Cameras and images | librealsense, OpenCV |
| Perception models | YOLOE, GraspGenX |
| Motion | ROS 2, MoveIt 2, Servo, MTC, Octomap |
| Persistence | Redis, service configuration and conversation-image storage |
| Deployment | Docker images with service-owned dependencies and runtimes |

Current hardware is the StarArm-102 six-axis arm, RA8-U35H-M servos and RealSense D415, with SDL3 gamepad and NOLO CV1 input adapters. Additional devices can be integrated through adapters; this is not a claim that every robot or camera is already supported.

After dependencies and weights are available, local segmentation, manual control and pick-and-place do not require an external language-model service. Natural-language interaction requires a usable model endpoint; internet requirements depend on where it is deployed.

The platform targets desktop robotics interaction and application development. Depth quality, calibration, mechanical play, gripper geometry and object material affect outcomes. Software completion is not substituted for physical grasp confirmation.

## Further reading

- [Architecture and call chains](docs/BACKEND.md)
- [Robot integration, servo parameters and accuracy](docs/STARARM-102.md)
- [Docker and service images](docs/DOCKER.md)

Technical reference documents are currently in Chinese. Screenshots show the running interface; the robot model is a hardware-feedback visualization. Manual annotations and unsent task examples are identified, and captured runtime values are not a list of default settings.
