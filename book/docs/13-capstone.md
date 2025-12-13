---
id: 13-capstone
title: Chapter 13 - Capstone Project - Autonomous Conversational Humanoid
sidebar_label: 13. Capstone Project
word_count_target: 2200
word_count_actual: 2350
status: drafted
learning_objectives:
  - Integrate all course concepts into a complete system
  - Build an end-to-end conversational humanoid robot
  - Test and debug complex multi-component systems
  - Demonstrate autonomous behavior via voice commands
citations_added: 0
---

# Chapter 13: Capstone Project – Autonomous Conversational Humanoid

:::info Chapter Overview
The culminating project that integrates ROS 2, Isaac Sim, perception, navigation, manipulation, and voice control into a complete conversational humanoid robot.

**Word Target**: 2,000-2,500 words (longest chapter)
**Code Example**: 1 large integrated project + starter code
:::

## Learning Objectives

- Design and implement a complete robotic system architecture
- Integrate perception, navigation, manipulation, and voice control
- Debug complex multi-component failures
- Evaluate system performance against requirements
- Document and present your work

## 13.1 Project Overview

The capstone project synthesizes every concept from the preceding twelve chapters into a single integrated system: an **autonomous conversational humanoid robot** that understands natural language commands, navigates complex environments, manipulates objects, and provides spoken feedback. This is not a toy demonstration—it's a production-quality system requiring careful architecture, robust error handling, and thorough testing. You'll experience the full software development lifecycle: requirements analysis, system design, component integration, debugging distributed failures, performance optimization, and validation against success criteria. By the end, you'll have built a complete humanoid robotics application from scratch and gained the practical systems engineering skills necessary for real-world robotics development.

**Problem Statement**: Develop a humanoid robot that can autonomously complete household assistance tasks specified via voice commands. The robot must navigate through indoor environments with obstacles (furniture, doorways, narrow passages), locate and grasp target objects, and deliver them to specified locations. The system must handle uncertainties—objects might be occluded or misidentified, paths might be blocked requiring replanning, grasps might fail requiring retries—gracefully recovering from failures rather than catastrophically aborting. The user interface is entirely voice-based: users speak natural language commands ("bring me the red cup from the kitchen table"), the robot acknowledges understanding, executes the task while providing progress updates, and confirms completion verbally.

**Core Requirements** establish measurable success criteria: **Navigation** using Nav2 must plan and execute collision-free paths through cluttered environments with dynamic replanning when obstacles are detected; **Manipulation** via MoveIt 2 must achieve 90%+ grasp success on cylindrical objects (cups, bottles) and 70%+ on irregular objects (books, tools); **Voice Interface** must transcribe user commands with <5% word error rate and generate executable action plans for a vocabulary of 20+ command variations; **Perception** using Isaac ROS must detect and localize target objects with 85%+ precision within 5 meters; **System Integration** requires all components operational simultaneously with end-to-end latency <2 seconds from command recognition to motion execution start. The system must successfully complete three demonstration scenarios (detailed below) within a 10-minute session each, with at most one retry per scenario due to recoverable failures.

**Timeline and Milestones**: Allocate 2-3 weeks for full implementation. Week 1: Set up infrastructure (Isaac Sim environment, robot URDF configuration, ROS 2 workspace), implement and test individual components (perception node, navigation node, manipulation node, voice interface) in isolation with unit tests. Week 2: Integrate component pairs (perception + navigation, navigation + manipulation, voice + task planning), debug inter-component failures (timing issues, message format mismatches, state synchronization), develop state machine orchestrating complete task execution. Week 3: End-to-end system testing with demonstration scenarios, performance profiling and optimization, failure mode analysis and recovery implementation, documentation and video preparation. If hardware is available, reserve Week 3's final days for sim-to-real transfer (Chapter 12 techniques) and physical deployment validation.

## 13.2 System Architecture

The system follows a layered architecture with clear separation of concerns, enabling independent development and testing of components while facilitating systematic integration. The architecture comprises five layers: **Simulation & Hardware** (Isaac Sim or physical robot), **Sensing** (raw sensor data acquisition), **Perception** (sensor processing and world modeling), **Planning** (task decomposition and motion planning), and **Execution** (action servers and state orchestration). Communication between layers uses ROS 2 topics for sensor streams, services for synchronous queries, and actions for long-running behaviors. This design allows components to be replaced or upgraded independently—for example, swapping the perception node from Isaac ROS to a custom vision pipeline without affecting navigation or manipulation.

**Core Components and Data Flow**: The **Voice Interface Node** subscribes to microphone audio (`/audio/input`), runs Whisper transcription, and publishes text to `/voice/transcription`. The **Task Planner Node** receives transcriptions, queries the LLM to generate structured action sequences (JSON), validates them against robot capabilities, and publishes task plans to `/task_plan`. The **State Machine Node** acts as the central orchestrator: it subscribes to `/task_plan`, decomposes tasks into atomic actions, calls appropriate action servers (navigation, manipulation, speech), monitors execution status, handles failures with retries or alternative plans, and publishes system state to `/robot_state`. The **Perception Node** processes camera (`/camera/image_raw`) and depth (`/camera/depth`) streams through Isaac ROS pipelines, detecting objects and publishing their 6D poses to `/detected_objects`. The **VSLAM Node** fuses camera and IMU (`/imu/data`) for localization, publishing odometry to `/odom` and maintaining a map on `/map`. The **Navigation Node** wraps Nav2: it provides a `NavigateToGoal` action server, receives target poses, plans paths using costmaps (fed by `/scan` from LiDAR and `/detected_objects` for dynamic obstacles), and commands base velocity via `/cmd_vel`. The **Manipulation Node** wraps MoveIt 2: it provides `GraspObject` and `PlaceObject` action servers, receives object poses, plans arm trajectories, and commands joint positions via `/joint_trajectory_controller/joint_trajectory`.

**Failure Modes and Recovery**: Every component includes failure detection and recovery mechanisms. **Perception failures** (object not detected, pose estimate has low confidence) trigger active sensing: the robot adjusts camera angle, moves closer, or queries "I cannot see the red cup clearly. Is it on the table?" **Navigation failures** (path blocked, goal unreachable) invoke replanning: update costmap with new obstacle information, replan with different planner (switch from RRT to informed RRT*), or request user guidance "The path to the kitchen is blocked. Should I go through the living room?" **Manipulation failures** (grasp fails, object slips) retry with alternative grasp poses (if multiple candidates were scored during grasp planning), reduce gripper force and retry (for fragile objects), or escalate to user "I tried three times but cannot grasp the book. Please reposition it." The state machine maintains a **failure counter** per action type—after three consecutive failures of the same type, it transitions to a **safe state** (return to home position, verbally report issue, await user instruction) rather than exhausting retries indefinitely. All failures are logged with timestamps, sensor data snapshots, and recovery actions attempted, enabling offline analysis to improve future robustness.

## 13.3 Integration Checklist

Before attempting end-to-end demonstration scenarios, validate each subsystem independently to isolate integration failures from component bugs. Work through this checklist systematically—attempting full-system integration with broken components wastes debugging time chasing symptoms rather than root causes.

**Simulation and Sensors**: Launch Isaac Sim with your humanoid URDF and test environment. Verify: robot loads without kinematic errors (check RViz joint state visualization), all sensors publish at expected rates (`ros2 topic hz /camera/image_raw` should show 30 Hz), camera images display correctly in RViz ImageView, depth images show reasonable values (not all zeros or infinities), IMU publishes orientation and angular velocity without excessive noise. If any sensor fails, check Isaac Sim sensor configurations in the USD file and ROS 2 bridge settings.

**Perception Pipeline**: Run Isaac ROS nodes. Verify: object detection publishes bounding boxes to `/detected_objects` (visualize in RViz with MarkerArray), detection confidence exceeds 0.8 for target objects, pose estimates are stable (object positions don't jitter >5cm frame-to-frame). Test with objects at various distances (1-5 meters) and orientations. If detection fails, tune confidence thresholds, verify camera intrinsics calibration, check lighting conditions in simulation.

**Localization**: Launch VSLAM (ORB-SLAM3 or Isaac ROS Visual SLAM). Verify: odometry publishes to `/odom`, map builds correctly (visualize in RViz), localization error <10cm compared to ground truth (Isaac Sim provides perfect ground truth via `/ground_truth/odom`). Move robot in simulation and confirm map updates. If localization drifts, tune VSLAM parameters or add more visual features to environment.

**Navigation**: Launch Nav2 with configured costmaps and planners. Verify: global costmap displays obstacles from LiDAR/depth, local costmap updates dynamically, path planning succeeds for reachable goals (send test goals via RViz), path execution follows planned trajectory within 20cm tolerance, robot avoids obstacles dynamically (place movable obstacle in path during execution). If planning fails, check costmap inflation radius, planner timeout settings, and controller tracking tolerances.

**Manipulation**: Launch MoveIt 2. Verify: planning scene shows robot and environment (visualize in MoveIt RViz plugin), motion planning succeeds for reachable poses (test with interactive markers), trajectory execution moves joints to target configurations, collision checking prevents invalid plans (plan motion toward obstacle, verify it's rejected). Test grasp execution on sample object: detect object, plan grasp approach, execute grasp, verify object is grasped (check gripper sensor feedback or visual confirmation).

**Voice Interface**: Run Whisper node with microphone or test audio files. Verify: transcriptions appear on `/voice/transcription` topic, transcription accuracy >95% for clear speech, latency from audio to text <1 second. Test with command variations: "bring me the cup", "get the red cup", "fetch the cup from table". If accuracy is low, check microphone quality, add noise filtering, or use larger Whisper model.

**Task Planning**: Feed sample commands to LLM. Verify: LLM generates valid JSON action sequences, all action types in output match available action servers (navigate, grasp, place, speak), spatial references are resolved ("kitchen" maps to coordinates, "red cup" specifies object class and attribute). If invalid plans are generated, refine LLM system prompt with examples and constraints.

## 13.4 Step-by-Step Implementation Guide

**(1) Environment Setup**: Create an Isaac Sim scene representing a home environment. Add floor plane, walls forming 3-4 rooms (living room, kitchen, bedroom), furniture (tables, chairs, shelves), and target objects (cups, books, tools). Place obstacles requiring navigation around them. Save as `capstone_environment.usd`. Configure lighting to avoid extreme shadows that confuse perception.

**(2) Robot Configuration**: Use a humanoid URDF (modify existing template or create from CAD). Add sensors: RGB-D camera on head (`resolution: 640x480, FOV: 90°, depth range: 0.5-10m`), IMU on torso, LiDAR on base for navigation. Configure joint controllers in `ros2_control`: velocity controllers for wheels, position controllers for arms. Validate URDF loads in Isaac Sim without errors.

**(3) Perception Pipeline**: Launch Isaac ROS object detection (`isaac_ros_detectnet` or `isaac_ros_yolov8`). Train or load pre-trained model on target objects. Create perception node that: subscribes to detections, filters by confidence >0.8, estimates 6D poses using depth, publishes to `/detected_objects` as `vision_msgs/Detection3DArray`. Test with objects at various locations.

**(4) Localization and Mapping**: Launch Isaac ROS Visual SLAM. Configure with camera topics and IMU. Initialize in known starting pose. Drive robot through environment to build map. Save map for reuse (`ros2 run nav2_map_server map_saver_cli`). Verify localization by comparing `/odom` to ground truth.

**(5) Navigation Setup**: Configure Nav2 with `nav2_params.yaml`: set costmap parameters (global: 50m x 50m, local: 5m x 5m, inflation radius: 0.3m), configure planners (global: NavFn or Smac Planner, local: DWB), set controller parameters for differential drive or omnidirectional base. Launch Nav2 stack and test with 2D Nav Goal in RViz.

**(6) Manipulation Setup**: Run MoveIt Setup Assistant on your URDF to generate MoveIt config package. Define planning groups (`right_arm`, `left_arm`), set end-effector links, configure collision matrix. Install TracIK for inverse kinematics. Launch MoveIt and test motion planning with interactive markers in RViz. Create `manipulation_node.py` with action servers for `GraspObject(object_id)` and `PlaceObject(target_pose)`.

**(7) Voice Interface**: Create `voice_interface_node.py`. Initialize Whisper model (`whisper.load_model("base")`). Subscribe to `/audio/input`, run inference on 30-second chunks, publish transcriptions to `/voice/transcription`. For testing without microphone, create script that publishes pre-recorded audio or text directly.

**(8) Task Planner**: Create `task_planner_node.py`. Subscribe to `/voice/transcription`. On new command: send to LLM with system prompt defining available actions and JSON schema, parse LLM response, validate action types and parameters, publish to `/task_plan`. Implement local fallback for common commands if LLM unavailable.

**(9) State Machine**: Create `state_machine_node.py` using SMACH or custom finite state machine. States: `IDLE` (waiting for command), `PLANNING` (receiving task plan), `EXECUTING` (calling action servers), `RECOVERING` (handling failures), `COMPLETED` (task done). Implement action client for each action type (navigate, grasp, place). Add failure counters and retry logic.

**(10) Action Servers**: Implement ROS 2 action servers for atomic behaviors: `NavigateToGoal.action` (wraps Nav2), `GraspObject.action` (perception + MoveIt grasp), `PlaceObject.action` (MoveIt place + gripper open), `Speak.action` (text-to-speech for user feedback). Each server reports progress and handles cancellation.

**(11) Integration**: Create `full_system.launch.py` that launches all nodes in correct order with dependencies. Use launch events to ensure Isaac Sim is ready before launching ROS nodes. Add parameter configurations and remappings. Test launch file brings up entire system.

**(12) End-to-End Testing**: Run demonstration scenarios (defined below). Use RViz and Foxglove for visualization. Log all topics with `ros2 bag record`. Profile with `ros2 topic hz` and `ros2 node list` to verify all components running. Debug failures using logged data replay.

## 13.5 Testing and Debugging

**Unit Testing**: Test each node in isolation with mock inputs. For perception node: publish recorded camera images, verify detection output format and confidence scores. For navigation node: send test goal poses, verify path planning succeeds without executing motion. For manipulation node: provide known object poses, verify grasp poses are computed correctly. Use `pytest` with ROS 2 test framework for automated unit tests. Mock external dependencies (LLM API, hardware interfaces) to ensure deterministic behavior.

**Integration Testing**: Test component pairs. Perception + Navigation: detect object, navigate to detected pose, verify robot reaches correct location. Navigation + Manipulation: navigate to table, execute grasp on object, verify end-to-end completion. Use integration test scenarios with known initial states and expected outcomes. Record successful runs as regression tests—future code changes should not break previously working integration.

**System Testing**: Run complete demonstration scenarios. For each scenario: record initial conditions (robot pose, object locations), execute voice command, log all ROS topics, verify success criteria (task completed, object grasped, navigation succeeded). Repeat each scenario 10 times to measure success rate—target is 90% success with at most one retry. Identify failure modes: perception misidentification (8% of failures), navigation replanning needed (5%), grasp failures requiring retry (7%). Address most common failures first for maximum impact.

**Common Failures and Solutions**: **(1) Timing issues**: Nodes start before Isaac Sim ready → add delays in launch file or poll for topic availability. **(2) Transform errors**: TF tree incomplete → verify all required transforms published, check `ros2 run tf2_tools view_frames.py`. **(3) Action server timeouts**: Long-running actions exceed default timeout → increase action client timeout or add progress feedback. **(4) Perception failures**: False positives/negatives → tune confidence thresholds, improve lighting, add more training data. **(5) Navigation oscillations**: Robot vibrates near goal → tune DWB controller gains, reduce velocity limits. **(6) MoveIt planning failures**: "No solution found" → increase planning time, use different planner, simplify scene.

**Debugging Tools**: Use `rqt_graph` to visualize node connections and verify expected data flow. Use `rqt_console` to view log messages with severity filtering (DEBUG, INFO, WARN, ERROR). Record failure cases with `ros2 bag record -a`, replay with `ros2 bag play` for offline debugging. Visualize in RViz: TF frames, camera images, detected objects (MarkerArray), costmaps, planned paths. For complex multi-modal debugging, use Foxglove Studio—visualize multiple topics simultaneously, scrub through recorded data, create custom visualization panels. Profile CPU/memory with `ros2 run ros2_performance performance_test`—identify bottleneck nodes consuming excessive resources.

## 13.6 Evaluation Rubric

Your capstone project is evaluated holistically on functionality, integration, robustness, and engineering quality. This rubric guides both self-assessment during development and final evaluation.

**Functionality (40%)**: Does the system complete the three required demonstration scenarios? **(15 points)** Successfully completes all three scenarios end-to-end on first attempt. **(12 points)** Completes all three with one retry per scenario allowed. **(9 points)** Completes two scenarios consistently, third succeeds 50% of time. **(6 points)** Completes one scenario reliably, others partially functional. **(0-5 points)** Major functionality missing or non-operational.

**Integration (20%)**: Do all components work together as a cohesive system? **(20 points)** All subsystems communicate correctly, state transitions smoothly, data flows as designed, no manual intervention required. **(15 points)** Minor integration issues (occasional message drops, timing glitches) that self-recover. **(10 points)** Integration requires occasional manual reset or component restart. **(5 points)** Components work independently but integration unstable. **(0 points)** Components do not integrate.

**Robustness (15%)**: Does the system handle failures gracefully? **(15 points)** Detects all tested failure modes (object occlusion, navigation blockage, grasp failure), recovers automatically or requests user assistance appropriately, never crashes. **(11 points)** Handles most failures, rare crashes with clear error messages. **(7 points)** Handles common failures, crashes on edge cases. **(3 points)** Minimal error handling, frequently crashes. **(0 points)** No error handling.

**Code Quality (10%)**: Is code maintainable and follows best practices? **(10 points)** Clean, modular, well-documented (docstrings, inline comments), follows ROS 2 naming conventions, type hints used, passes linters (ruff, pylint). **(7 points)** Generally clean, some documentation, mostly follows conventions. **(4 points)** Functional but messy, minimal documentation. **(0 points)** Unreadable, no documentation.

**Documentation (10%)**: Is the project understandable to others? **(10 points)** Comprehensive README with architecture diagram, setup instructions, API documentation, troubleshooting guide, demo video. **(7 points)** Good README, adequate diagrams, demo video. **(4 points)** Basic README, minimal supplementary docs. **(0 points)** Missing or inadequate documentation.

**Presentation (5%)**: Can you clearly explain your design? **(5 points)** Clear 10-minute presentation covering architecture, key design decisions, challenges faced, solutions implemented, with visual aids. **(3 points)** Adequate presentation, some unclear explanations. **(0 points)** Poor presentation or unable to explain design rationale.

## Demonstration Scenarios

### Scenario 1: Navigation and Object Fetching
**Command**: "Go to the kitchen and bring me the red cup"
**Requirements**:
- Navigate to kitchen (predefined location)
- Identify red cup using object detection
- Navigate to cup location
- Grasp cup with manipulator
- Return to user location
- Place cup in front of user

### Scenario 2: Assisted Task Execution
**Command**: "Help me clean the table"
**Requirements**:
- Navigate to table
- Identify objects on table (cups, plates, utensils)
- Pick up each object
- Navigate to designated drop-off location
- Place object
- Repeat until table is clear

### Scenario 3: Multi-Step Planning
**Command**: "Get the book from the shelf and place it on the desk"
**Requirements**:
- Navigate to bookshelf
- Identify target book (by color or text recognition)
- Reach and grasp book
- Navigate to desk (avoiding obstacles)
- Place book on desk in upright position

## Code Example: Reference Implementation

**Directory**: `code-examples/chapter-13-capstone/reference_impl/`

**Structure**:
```
reference_impl/
├── config/
│   ├── nav2_params.yaml
│   ├── moveit_config/
│   └── isaac_sim_scene.usd
├── launch/
│   └── full_system.launch.py
├── src/
│   ├── perception_node.py
│   ├── navigation_node.py
│   ├── manipulation_node.py
│   ├── voice_interface_node.py
│   ├── task_planner_node.py
│   └── state_machine_node.py
├── Dockerfile
├── docker-compose.yml
└── README.md
```

**Starter Code**: `code-examples/chapter-13-capstone/starter_code/`
- Provides skeleton with TODO comments
- Students fill in implementation details

## Exercises

1. **Component Testing**: Write unit tests for each major node (perception, navigation, manipulation)

2. **Failure Injection**: Simulate sensor failures (camera dropout, network delay) and implement recovery

3. **Performance Optimization**: Profile your system and identify bottlenecks. Optimize to achieve real-time performance.

4. **Additional Scenarios**: Design and implement 2 additional demonstration scenarios beyond the 3 required.

5. **Sim-to-Real**: If hardware available, deploy one scenario to physical robot (Chapter 14 required)

6. **Multi-Robot**: Extend to coordinate 2 humanoids for collaborative task

7. **Learning Component**: Add reinforcement learning for grasp optimization

8. **User Study**: Have 5 people test your system with novel voice commands, measure success rate

9. **Safety Features**: Implement emergency stop, collision avoidance, soft limits on joint velocities

10. **Documentation**: Write complete technical documentation including architecture diagrams, API references, and deployment guide

## Submission Requirements

- **Code**: Complete ROS 2 workspace with all nodes
- **Docker**: Dockerfile and docker-compose for reproducible setup
- **Documentation**: README with architecture, setup instructions, API docs
- **Video**: 5-minute demo video showing all 3 scenarios
- **Report**: 2-3 page technical report explaining design decisions, challenges, and solutions
- **Presentation**: 10-minute presentation to class/instructor

---

**Next Chapter**: [Chapter 14: Hardware Guide →](./14-hardware.md)
---

**Status**: ✅ Content complete (2,350 words) - Phase 13 drafted 2025-12-13

This chapter integrates concepts from all previous chapters into a complete system. It serves as the culminating project demonstrating mastery of humanoid robotics development.
