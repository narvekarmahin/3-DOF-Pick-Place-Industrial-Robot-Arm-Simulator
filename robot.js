/**
 * robot.js
 * 
 * Three.js 3D Industrial Robot Arm Visualizer & Scene Graph Engine.
 * Constructs realistic 3D meshes, hierarchical kinematic joints, parallel-jaw gripper,
 * workpieces, target markers, lighting, and real-time trajectory path tracing.
 */

(function (global) {
  'use strict';

  class RobotVisualizer {
    constructor(canvasContainerId, options = {}) {
      this.container = document.getElementById(canvasContainerId);
      if (!this.container) {
        throw new Error(`Container #${canvasContainerId} not found.`);
      }

      this.options = {
        theme: 'orange', // 'orange', 'yellow', 'white', 'cyan'
        traceTrail: true,
        maxTrailPoints: 600,
        ...options
      };

      // Kinematic link lengths matching kinematics.js
      this.d1 = 1.5; // Link 1 height (m)
      this.a2 = 1.2; // Link 2 length (m)
      this.a3 = 0.8; // Link 3 length (m)

      // Target Locations
      this.targetAPos = new THREE.Vector3(1.0, 1.0, 0.2);
      this.targetBPos = new THREE.Vector3(-1.0, 1.2, 0.2);

      // State
      this.currentAngles = { theta1: 0, theta2: Math.PI / 4, theta3: -Math.PI / 4 };
      this.gripperOpenRatio = 1.0; // 1 = open, 0 = closed
      this.holdingCube = false;
      this.trailPoints = [];

      this.initScene();
      this.initLights();
      this.initEnvironment();
      this.initRobotModel();
      this.initWorkpieces();
      this.initTrajectoryTracer();

      // Apply initial Home angles
      this.setJointAngles(this.currentAngles.theta1, this.currentAngles.theta2, this.currentAngles.theta3);

      this.bindEvents();
      this.animate = this.animate.bind(this);
      this.render();
    }

    initScene() {
      // Configure Three.js world with Z-axis as UP (Robotics standard: Z = UP, XY = Floor)
      THREE.Object3D.DefaultUp.set(0, 0, 1);

      this.scene = new THREE.Scene();
      this.scene.background = new THREE.Color(0x0e1117);
      this.scene.fog = new THREE.FogExp2(0x0e1117, 0.045);

      const width = this.container.clientWidth || window.innerWidth;
      const height = this.container.clientHeight || window.innerHeight;

      this.camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
      this.camera.up.set(0, 0, 1);
      // Position camera at an isometric viewpoint
      this.camera.position.set(3.8, -4.5, 3.2);

      this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
      this.renderer.setSize(width, height);
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
      this.renderer.shadowMap.enabled = true;
      this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
      this.renderer.toneMappingExposure = 1.1;

      this.container.appendChild(this.renderer.domElement);

      // Orbit Controls
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.06;
      this.controls.target.set(0, 0, 1.2); // Focus around robot center
      this.controls.maxPolarAngle = Math.PI / 2 + 0.05; // Don't dip far below ground
      this.controls.minDistance = 1.0;
      this.controls.maxDistance = 18.0;
    }

    initLights() {
      // Hemisphere Light (sky bounce and ground bounce)
      const hemiLight = new THREE.HemisphereLight(0xddeeff, 0x1a202c, 0.6);
      hemiLight.position.set(0, 0, 20);
      this.scene.add(hemiLight);

      // Main Key Directional Light with soft shadows
      const keyLight = new THREE.DirectionalLight(0xffffff, 1.2);
      keyLight.position.set(5, -6, 8);
      keyLight.castShadow = true;
      keyLight.shadow.mapSize.width = 2048;
      keyLight.shadow.mapSize.height = 2048;
      keyLight.shadow.camera.near = 0.5;
      keyLight.shadow.camera.far = 25;
      const d = 3.5;
      keyLight.shadow.camera.left = -d;
      keyLight.shadow.camera.right = d;
      keyLight.shadow.camera.top = d;
      keyLight.shadow.camera.bottom = -d;
      keyLight.shadow.bias = -0.0003;
      this.scene.add(keyLight);

      // Cool Blue Fill Light
      const fillLight = new THREE.DirectionalLight(0x70a1ff, 0.5);
      fillLight.position.set(-6, 6, 6);
      this.scene.add(fillLight);

      // Warm Rim / Accent Light
      const rimLight = new THREE.PointLight(0xffbe76, 0.8, 15);
      rimLight.position.set(-2, -3, 4);
      this.scene.add(rimLight);
    }

    initEnvironment() {
      // Ground plane with subtle dark industrial finish
      const floorGeo = new THREE.PlaneGeometry(16, 16);
      const floorMat = new THREE.MeshStandardMaterial({
        color: 0x131720,
        roughness: 0.85,
        metalness: 0.2
      });
      this.floorMesh = new THREE.Mesh(floorGeo, floorMat);
      this.floorMesh.receiveShadow = true;
      this.scene.add(this.floorMesh);

      // Dark Industrial Grid on XY plane (Z=0)
      this.gridHelper = new THREE.GridHelper(12, 24, 0x00d2d3, 0x222d3d);
      this.gridHelper.rotation.x = Math.PI / 2; // Orient to XY plane
      this.gridHelper.position.z = 0.002;
      this.scene.add(this.gridHelper);

      // Concentric workspace boundary circle at max reach (R = 2.0m + 0.5m)
      const circleGeo = new THREE.RingGeometry(1.99, 2.01, 64);
      const circleMat = new THREE.MeshBasicMaterial({
        color: 0xff3838,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.35
      });
      this.reachLimitCircle = new THREE.Mesh(circleGeo, circleMat);
      this.reachLimitCircle.position.z = 0.004;
      this.scene.add(this.reachLimitCircle);

      // Origin RGB Coordinate Axes (Red = X, Green = Y, Blue = Z)
      this.originAxes = new THREE.AxesHelper(1.2);
      this.originAxes.position.z = 0.005;
      this.originAxes.material.linewidth = 3;
      this.originAxes.renderOrder = 2;
      this.scene.add(this.originAxes);
    }

    createMaterials() {
      // Palette definitions
      const armColors = {
        orange: 0xf368e0 ? 0xff6b08 : 0xff793f, // Industrial KUKA orange
        yellow: 0xf1c40f,                       // FANUC Yellow
        white: 0xecf0f1,                        // ABB Modern White
        cyan: 0x00d2d3                          // Tech Cyan
      };

      const primaryColor = armColors[this.options.theme] || 0xff6b08;

      this.materials = {
        baseCastIron: new THREE.MeshStandardMaterial({
          color: 0x1e242c,
          roughness: 0.5,
          metalness: 0.75
        }),
        robotArm: new THREE.MeshStandardMaterial({
          color: primaryColor,
          roughness: 0.3,
          metalness: 0.55
        }),
        jointAlloy: new THREE.MeshStandardMaterial({
          color: 0x4b5563,
          roughness: 0.25,
          metalness: 0.85
        }),
        accentDark: new THREE.MeshStandardMaterial({
          color: 0x111827,
          roughness: 0.4,
          metalness: 0.6
        }),
        gripperSteel: new THREE.MeshStandardMaterial({
          color: 0x9ca3af,
          roughness: 0.2,
          metalness: 0.9
        }),
        gripperPad: new THREE.MeshStandardMaterial({
          color: 0xef4444,
          roughness: 0.9,
          metalness: 0.1
        }),
        ledActive: new THREE.MeshBasicMaterial({
          color: 0x10b981
        }),
        ledLaser: new THREE.MeshBasicMaterial({
          color: 0xff0055,
          transparent: true,
          opacity: 0.6
        })
      };
    }

    initRobotModel() {
      this.createMaterials();

      // Root robot group at world origin
      this.robotRoot = new THREE.Group();
      this.robotRoot.name = "Robot_Root";
      this.scene.add(this.robotRoot);

      // 1. Static Pedestal Base on Ground
      const basePedestal = new THREE.Group();
      
      // Bottom circular mounting flange
      const basePlateGeo = new THREE.CylinderGeometry(0.55, 0.6, 0.08, 36);
      basePlateGeo.rotateX(Math.PI / 2); // Cylinder along Z
      const basePlate = new THREE.Mesh(basePlateGeo, this.materials.baseCastIron);
      basePlate.position.z = 0.04;
      basePlate.castShadow = true;
      basePlate.receiveShadow = true;
      basePedestal.add(basePlate);

      // Chamfered pedestal body
      const pedestalGeo = new THREE.CylinderGeometry(0.38, 0.5, 0.22, 32);
      pedestalGeo.rotateX(Math.PI / 2);
      const pedestal = new THREE.Mesh(pedestalGeo, this.materials.baseCastIron);
      pedestal.position.z = 0.19;
      pedestal.castShadow = true;
      basePedestal.add(pedestal);

      // Safety warning ring around pedestal base
      const hazardRingGeo = new THREE.TorusGeometry(0.56, 0.02, 16, 40);
      const hazardRingMat = new THREE.MeshBasicMaterial({ color: 0xf59e0b });
      const hazardRing = new THREE.Mesh(hazardRingGeo, hazardRingMat);
      hazardRing.position.z = 0.08;
      basePedestal.add(hazardRing);

      this.robotRoot.add(basePedestal);

      // ==========================================
      // 2. JOINT 1: Base Rotation around Z (θ1)
      // ==========================================
      this.joint1Group = new THREE.Group();
      this.joint1Group.name = "Joint1_BaseZ";
      this.joint1Group.position.set(0, 0, 0);
      this.robotRoot.add(this.joint1Group);

      // Turret rotating turntable (Z = 0.3 to 0.4)
      const turretGeo = new THREE.CylinderGeometry(0.35, 0.36, 0.12, 32);
      turretGeo.rotateX(Math.PI / 2);
      const turret = new THREE.Mesh(turretGeo, this.materials.jointAlloy);
      turret.position.z = 0.36;
      turret.castShadow = true;
      this.joint1Group.add(turret);

      // LINK 1: Vertical Column (Height d1 = 1.5m, from Z = 0.4 to Z = 1.5)
      const colHeight = this.d1 - 0.4; // 1.1m
      const colGeo = new THREE.CylinderGeometry(0.24, 0.28, colHeight, 32);
      colGeo.rotateX(Math.PI / 2);
      const column = new THREE.Mesh(colGeo, this.materials.robotArm);
      column.position.z = 0.4 + colHeight / 2;
      column.castShadow = true;
      this.joint1Group.add(column);

      // Vertical column accent flutes & spine detail
      const spineGeo = new THREE.BoxGeometry(0.12, 0.32, colHeight * 0.85);
      const spine = new THREE.Mesh(spineGeo, this.materials.accentDark);
      spine.position.set(0.18, 0, 0.4 + colHeight / 2);
      spine.castShadow = true;
      this.joint1Group.add(spine);

      // Shoulder Support Yoke at Z = 1.5m
      const yokeHubGeo = new THREE.CylinderGeometry(0.28, 0.28, 0.26, 32);
      // Axis along Y for shoulder housing
      const yokeHub = new THREE.Mesh(yokeHubGeo, this.materials.jointAlloy);
      yokeHub.position.set(0, 0, this.d1);
      yokeHub.castShadow = true;
      this.joint1Group.add(yokeHub);

      // ==========================================
      // 3. JOINT 2: Shoulder Rotation around local Y (θ2)
      // Shoulder pivot location is at [0, 0, d1 = 1.5m]
      // ==========================================
      this.joint2Group = new THREE.Group();
      this.joint2Group.name = "Joint2_ShoulderY";
      this.joint2Group.position.set(0, 0, this.d1);
      this.joint1Group.add(this.joint2Group);

      // Shoulder cylindrical joint hub (along Y axis)
      const shoulderCapGeo = new THREE.CylinderGeometry(0.25, 0.25, 0.32, 28);
      const shoulderCap = new THREE.Mesh(shoulderCapGeo, this.materials.accentDark);
      shoulderCap.castShadow = true;
      this.joint2Group.add(shoulderCap);

      // Shoulder servo encoder disc with status light
      const encoderGeo = new THREE.CylinderGeometry(0.14, 0.14, 0.34, 20);
      const encoder = new THREE.Mesh(encoderGeo, this.materials.jointAlloy);
      this.joint2Group.add(encoder);

      // Counterweight extension behind shoulder pivot (-X direction)
      const counterWeightGeo = new THREE.BoxGeometry(0.35, 0.26, 0.28);
      const counterWeight = new THREE.Mesh(counterWeightGeo, this.materials.baseCastIron);
      counterWeight.position.set(-0.25, 0, 0);
      counterWeight.castShadow = true;
      this.joint2Group.add(counterWeight);

      // LINK 2: Upper Arm Boom (Length a2 = 1.2m along local +X)
      // Constructed from local [0, 0, 0] to [1.2, 0, 0]
      const link2Length = this.a2;
      const upperArmBodyGeo = new THREE.BoxGeometry(link2Length - 0.2, 0.18, 0.22);
      const upperArmBody = new THREE.Mesh(upperArmBodyGeo, this.materials.robotArm);
      upperArmBody.position.set(link2Length / 2, 0, 0);
      upperArmBody.castShadow = true;
      this.joint2Group.add(upperArmBody);

      // Decorative reinforcement trusses & brand plate along Link 2
      const trussGeo = new THREE.CylinderGeometry(0.04, 0.04, link2Length - 0.3, 16);
      trussGeo.rotateZ(Math.PI / 2);
      const trussTop = new THREE.Mesh(trussGeo, this.materials.jointAlloy);
      trussTop.position.set(link2Length / 2, 0.1, 0.1);
      trussTop.castShadow = true;
      this.joint2Group.add(trussTop);

      const trussBottom = new THREE.Mesh(trussGeo, this.materials.jointAlloy);
      trussBottom.position.set(link2Length / 2, -0.1, 0.1);
      trussBottom.castShadow = true;
      this.joint2Group.add(trussBottom);

      // ==========================================
      // 4. JOINT 3: Elbow Rotation around local Y (θ3)
      // Elbow pivot location is at [a2 = 1.2, 0, 0] relative to Joint 2
      // ==========================================
      this.joint3Group = new THREE.Group();
      this.joint3Group.name = "Joint3_ElbowY";
      this.joint3Group.position.set(this.a2, 0, 0);
      this.joint2Group.add(this.joint3Group);

      // Elbow cylindrical joint hub (along Y axis)
      const elbowHubGeo = new THREE.CylinderGeometry(0.2, 0.2, 0.28, 28);
      const elbowHub = new THREE.Mesh(elbowHubGeo, this.materials.jointAlloy);
      elbowHub.castShadow = true;
      this.joint3Group.add(elbowHub);

      // Servomotor cooling fins on elbow joint
      const elbowMotorGeo = new THREE.CylinderGeometry(0.12, 0.12, 0.32, 20);
      const elbowMotor = new THREE.Mesh(elbowMotorGeo, this.materials.accentDark);
      this.joint3Group.add(elbowMotor);

      // LINK 3: Forearm (Length a3 = 0.8m along local +X)
      // Constructed from local [0, 0, 0] to [0.8, 0, 0]
      const link3Length = this.a3;
      const forearmGeo = new THREE.BoxGeometry(link3Length - 0.15, 0.14, 0.16);
      const forearm = new THREE.Mesh(forearmGeo, this.materials.robotArm);
      forearm.position.set(link3Length / 2 - 0.02, 0, 0);
      forearm.castShadow = true;
      this.joint3Group.add(forearm);

      // Tapered wrist housing approaching tool flange
      const wristGeo = new THREE.CylinderGeometry(0.08, 0.1, 0.12, 24);
      wristGeo.rotateZ(Math.PI / 2);
      const wrist = new THREE.Mesh(wristGeo, this.materials.jointAlloy);
      wrist.position.set(link3Length - 0.06, 0, 0);
      wrist.castShadow = true;
      this.joint3Group.add(wrist);

      // ==========================================
      // 5. END-EFFECTOR: Tool Flange & Parallel Gripper
      // Located at the tip of Link 3: [a3 = 0.8, 0, 0]
      // ==========================================
      this.endEffectorGroup = new THREE.Group();
      this.endEffectorGroup.name = "EndEffector_TCP";
      this.endEffectorGroup.position.set(this.a3, 0, 0);
      this.joint3Group.add(this.endEffectorGroup);

      // Circular tool mounting plate
      const flangeGeo = new THREE.CylinderGeometry(0.09, 0.09, 0.03, 24);
      flangeGeo.rotateZ(Math.PI / 2);
      const flange = new THREE.Mesh(flangeGeo, this.materials.jointAlloy);
      flange.position.set(0.015, 0, 0);
      this.endEffectorGroup.add(flange);

      // Gripper actuator chassis
      const gripperChassisGeo = new THREE.BoxGeometry(0.08, 0.14, 0.1);
      const gripperChassis = new THREE.Mesh(gripperChassisGeo, this.materials.accentDark);
      gripperChassis.position.set(0.07, 0, 0);
      gripperChassis.castShadow = true;
      this.endEffectorGroup.add(gripperChassis);

      // Status LED indicator on gripper chassis
      const ledGeo = new THREE.SphereGeometry(0.015, 12, 12);
      this.gripperLed = new THREE.Mesh(ledGeo, this.materials.ledActive);
      this.gripperLed.position.set(0.07, 0, 0.055);
      this.endEffectorGroup.add(this.gripperLed);

      // Gripper Fingers (Parallel Jaw Gripper)
      // Left and Right fingers translate along local Y axis
      this.fingerMaxTravel = 0.065; // half-stroke
      const fingerGeo = new THREE.BoxGeometry(0.09, 0.018, 0.04);

      // Left Finger Assembly
      this.leftFinger = new THREE.Group();
      this.leftFinger.position.set(0.12, this.fingerMaxTravel, 0);
      const leftFingerMesh = new THREE.Mesh(fingerGeo, this.materials.gripperSteel);
      leftFingerMesh.castShadow = true;
      this.leftFinger.add(leftFingerMesh);
      // Soft rubber grip pad on inner face
      const padGeo = new THREE.BoxGeometry(0.08, 0.005, 0.035);
      const leftPad = new THREE.Mesh(padGeo, this.materials.gripperPad);
      leftPad.position.set(0, -0.01, 0);
      this.leftFinger.add(leftPad);
      this.endEffectorGroup.add(this.leftFinger);

      // Right Finger Assembly
      this.rightFinger = new THREE.Group();
      this.rightFinger.position.set(0.12, -this.fingerMaxTravel, 0);
      const rightFingerMesh = new THREE.Mesh(fingerGeo, this.materials.gripperSteel);
      rightFingerMesh.castShadow = true;
      this.rightFinger.add(rightFingerMesh);
      const rightPad = new THREE.Mesh(padGeo, this.materials.gripperPad);
      rightPad.position.set(0, 0.01, 0);
      this.rightFinger.add(rightPad);
      this.endEffectorGroup.add(this.rightFinger);

      // TCP (Tool Center Point) Marker: Located at tip between fingers [0.12, 0, 0]
      this.tcpGroup = new THREE.Group();
      this.tcpGroup.name = "TCP_Frame";
      this.tcpGroup.position.set(0.12, 0, 0);
      this.endEffectorGroup.add(this.tcpGroup);

      // Small RGB Coordinate Frame for End-Effector TCP
      this.tcpAxes = new THREE.AxesHelper(0.25);
      this.tcpAxes.renderOrder = 3;
      this.tcpAxes.material.depthTest = false;
      this.tcpGroup.add(this.tcpAxes);

      // Laser guide beam pointing forward from TCP
      const laserGeo = new THREE.CylinderGeometry(0.002, 0.002, 0.4, 8);
      laserGeo.rotateZ(Math.PI / 2);
      this.laserBeam = new THREE.Mesh(laserGeo, this.materials.ledLaser);
      this.laserBeam.position.set(0.2, 0, 0);
      this.tcpGroup.add(this.laserBeam);
    }

    initWorkpieces() {
      // ==========================================
      // Target A: Pick Location [1.0, 1.0, 0.2]
      // Yellow Cube Workpiece
      // ==========================================
      this.targetAGroup = new THREE.Group();
      this.targetAGroup.position.copy(this.targetAPos);
      this.scene.add(this.targetAGroup);

      // Floor marker pedestal for Target A
      const padAGeo = new THREE.CylinderGeometry(0.3, 0.34, 0.04, 32);
      padAGeo.rotateX(Math.PI / 2);
      const padAMat = new THREE.MeshStandardMaterial({
        color: 0x332a00,
        roughness: 0.6,
        metalness: 0.4
      });
      const padA = new THREE.Mesh(padAGeo, padAMat);
      padA.position.set(this.targetAPos.x, this.targetAPos.y, 0.02);
      padA.receiveShadow = true;
      this.scene.add(padA);

      // Glowing targeting ring for A
      const ringAGeo = new THREE.RingGeometry(0.25, 0.28, 36);
      const ringAMat = new THREE.MeshBasicMaterial({
        color: 0xf1c40f,
        side: THREE.DoubleSide
      });
      const ringA = new THREE.Mesh(ringAGeo, ringAMat);
      ringA.position.set(this.targetAPos.x, this.targetAPos.y, 0.042);
      this.scene.add(ringA);

      // Workpiece: Yellow Chamfered Cube (0.16m x 0.16m x 0.16m)
      const cubeGeo = new THREE.BoxGeometry(0.16, 0.16, 0.16);
      this.cubeMaterial = new THREE.MeshStandardMaterial({
        color: 0xf1c40f, // Bright safety yellow
        roughness: 0.25,
        metalness: 0.45
      });
      this.cubeMesh = new THREE.Mesh(cubeGeo, this.cubeMaterial);
      this.cubeMesh.castShadow = true;
      this.cubeMesh.receiveShadow = true;

      // Initial cube location sits resting at Target A: [1.0, 1.0, 0.2]
      this.cubeInitialPos = new THREE.Vector3(1.0, 1.0, 0.2);
      this.cubeMesh.position.copy(this.cubeInitialPos);
      this.scene.add(this.cubeMesh);

      // Top gripping handle detail on cube
      const handleGeo = new THREE.BoxGeometry(0.04, 0.08, 0.02);
      const handleMat = new THREE.MeshStandardMaterial({ color: 0x222222, metalness: 0.8 });
      const handle = new THREE.Mesh(handleGeo, handleMat);
      handle.position.set(0, 0, 0.09);
      this.cubeMesh.add(handle);

      // ==========================================
      // Target B: Place Location [-1.0, 1.2, 0.2]
      // Green Industrial Drop Box / Bin
      // ==========================================
      this.targetBGroup = new THREE.Group();
      this.targetBGroup.position.copy(this.targetBPos);
      this.scene.add(this.targetBGroup);

      // Floor marker pedestal for Target B
      const padB = new THREE.Mesh(padAGeo, new THREE.MeshStandardMaterial({
        color: 0x073b1e,
        roughness: 0.6,
        metalness: 0.4
      }));
      padB.position.set(this.targetBPos.x, this.targetBPos.y, 0.02);
      padB.receiveShadow = true;
      this.scene.add(padB);

      // Glowing targeting ring for B
      const ringBGeo = new THREE.RingGeometry(0.3, 0.33, 36);
      const ringBMat = new THREE.MeshBasicMaterial({
        color: 0x2ecc71,
        side: THREE.DoubleSide
      });
      const ringB = new THREE.Mesh(ringBGeo, ringBMat);
      ringB.position.set(this.targetBPos.x, this.targetBPos.y, 0.042);
      this.scene.add(ringB);

      // Green Drop Box Receptacle
      // Open-top container: floor + 4 walls
      this.dropBoxGroup = new THREE.Group();
      this.dropBoxGroup.position.set(this.targetBPos.x, this.targetBPos.y, 0.04);
      this.scene.add(this.dropBoxGroup);

      const boxMat = new THREE.MeshStandardMaterial({
        color: 0x27ae60, // Emerald Green
        roughness: 0.35,
        metalness: 0.5
      });
      const boxRimMat = new THREE.MeshStandardMaterial({
        color: 0x2ecc71,
        roughness: 0.2,
        metalness: 0.8
      });

      const boxW = 0.42;
      const boxD = 0.42;
      const boxH = 0.28;
      const wallT = 0.03;

      // Bottom plate
      const bBottomGeo = new THREE.BoxGeometry(boxW, boxD, wallT);
      const bBottom = new THREE.Mesh(bBottomGeo, boxMat);
      bBottom.position.z = wallT / 2;
      bBottom.receiveShadow = true;
      this.dropBoxGroup.add(bBottom);

      // Wall 1: Front
      const wallFGeo = new THREE.BoxGeometry(boxW, wallT, boxH);
      const wallF = new THREE.Mesh(wallFGeo, boxMat);
      wallF.position.set(0, -boxD / 2 + wallT / 2, boxH / 2);
      wallF.castShadow = true;
      this.dropBoxGroup.add(wallF);

      // Wall 2: Back
      const wallB = new THREE.Mesh(wallFGeo, boxMat);
      wallB.position.set(0, boxD / 2 - wallT / 2, boxH / 2);
      wallB.castShadow = true;
      this.dropBoxGroup.add(wallB);

      // Wall 3: Left
      const wallLGeo = new THREE.BoxGeometry(wallT, boxD - 2 * wallT, boxH);
      const wallL = new THREE.Mesh(wallLGeo, boxMat);
      wallL.position.set(-boxW / 2 + wallT / 2, 0, boxH / 2);
      wallL.castShadow = true;
      this.dropBoxGroup.add(wallL);

      // Wall 4: Right
      const wallR = new THREE.Mesh(wallLGeo, boxMat);
      wallR.position.set(boxW / 2 - wallT / 2, 0, boxH / 2);
      wallR.castShadow = true;
      this.dropBoxGroup.add(wallR);

      // Detection sensor optical beam across top of drop box
      const beamGeo = new THREE.CylinderGeometry(0.003, 0.003, boxW, 8);
      this.dropSensorBeam = new THREE.Mesh(beamGeo, new THREE.MeshBasicMaterial({
        color: 0x2ecc71,
        transparent: true,
        opacity: 0.7
      }));
      this.dropSensorBeam.position.set(0, 0, boxH - 0.03);
      this.dropBoxGroup.add(this.dropSensorBeam);
    }

    initTrajectoryTracer() {
      // Dynamic Ribbon / Trail for End-Effector Motion
      const maxPts = this.options.maxTrailPoints;
      this.trailGeometry = new THREE.BufferGeometry();
      const positions = new Float32Array(maxPts * 3);
      const colors = new Float32Array(maxPts * 3);

      this.trailGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      this.trailGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));

      this.trailMaterial = new THREE.LineBasicMaterial({
        vertexColors: true,
        transparent: true,
        opacity: 0.85,
        linewidth: 3
      });

      this.trailLine = new THREE.Line(this.trailGeometry, this.trailMaterial);
      this.trailLine.frustumCulled = false;
      this.scene.add(this.trailLine);
    }

    /**
     * Set Robot Joint Rotations (θ1, θ2, θ3)
     * Coordinates & directions aligned with Kinematics engine:
     * - θ1 around Z-axis (turret azimuth)
     * - θ2 around local -Y axis (shoulder pitch: +θ2 tilts upward toward +Z)
     * - θ3 around local -Y axis (elbow pitch: +θ3 tilts upward, -θ3 tilts downward)
     */
    setJointAngles(theta1, theta2, theta3) {
      this.currentAngles.theta1 = theta1;
      this.currentAngles.theta2 = theta2;
      this.currentAngles.theta3 = theta3;

      // Apply rotations to Three.js joint hierarchy
      if (this.joint1Group) {
        this.joint1Group.rotation.z = theta1;
      }
      if (this.joint2Group) {
        this.joint2Group.rotation.y = -theta2;
      }
      if (this.joint3Group) {
        this.joint3Group.rotation.y = -theta3;
      }

      // Update scene matrices
      this.scene.updateMatrixWorld(true);

      // Record trajectory path point if tracing enabled
      if (this.options.traceTrail) {
        const tcpPos = this.getEndEffectorWorldPosition();
        this.addTrailPoint(tcpPos);
      }
    }

    /**
     * Control Gripper Jaw Open / Close Ratio
     * @param {number} ratio 1.0 = fully open, 0.0 = fully closed
     */
    setGripper(ratio) {
      this.gripperOpenRatio = Math.max(0, Math.min(1, ratio));
      // Finger travel: closed = 0.04 (gripping 0.16m cube), open = 0.075
      const closedPos = 0.042;
      const openPos = 0.078;
      const currentTravel = closedPos + (openPos - closedPos) * this.gripperOpenRatio;

      if (this.leftFinger && this.rightFinger) {
        this.leftFinger.position.y = currentTravel;
        this.rightFinger.position.y = -currentTravel;
      }

      // Update gripper status LED color
      if (this.gripperLed) {
        if (this.holdingCube) {
          this.gripperLed.material.color.setHex(0x10b981); // Green when holding
        } else if (this.gripperOpenRatio < 0.2) {
          this.gripperLed.material.color.setHex(0x3b82f6); // Blue when clamped empty
        } else {
          this.gripperLed.material.color.setHex(0xf59e0b); // Amber when ready/open
        }
      }
    }

    /**
     * Attach the workpiece cube mesh to the robot gripper tip (Pick action)
     */
    attachCubeToGripper() {
      if (this.holdingCube) return;
      this.holdingCube = true;

      // Detach cube from scene and attach to endEffector TCP
      THREE.SceneUtils.detach(this.cubeMesh, this.scene, this.scene);
      this.tcpGroup.add(this.cubeMesh);
      // Center cube flush inside gripper jaws
      this.cubeMesh.position.set(0.04, 0, 0);
      this.cubeMesh.rotation.set(0, 0, 0);

      this.setGripper(0.0); // Close fingers onto cube
    }

    /**
     * Detach the workpiece cube mesh from gripper into world space (Place action)
     */
    detachCubeToWorld(dropPosition) {
      if (!this.holdingCube) return;
      this.holdingCube = false;

      // Re-parent cube to world scene
      this.scene.add(this.cubeMesh);
      if (dropPosition) {
        this.cubeMesh.position.copy(dropPosition);
      } else {
        // Fallback: Place inside Target B drop box
        this.cubeMesh.position.set(this.targetBPos.x, this.targetBPos.y, 0.16);
      }
      this.cubeMesh.rotation.set(0, 0, 0);

      this.setGripper(1.0); // Open fingers to release
    }

    /**
     * Reset workpiece cube back to Target A pick position
     */
    resetCubeToTargetA() {
      if (this.holdingCube) {
        this.scene.add(this.cubeMesh);
        this.holdingCube = false;
      }
      this.cubeMesh.position.copy(this.cubeInitialPos);
      this.cubeMesh.rotation.set(0, 0, 0);
      this.setGripper(1.0);
    }

    /**
     * Get End-Effector World Position (X, Y, Z) directly from Three.js scene graph
     */
    getEndEffectorWorldPosition() {
      const vec = new THREE.Vector3();
      if (this.endEffectorGroup) {
        this.endEffectorGroup.getWorldPosition(vec);
      }
      return vec;
    }

    /**
     * Append a point to the dynamic trajectory tracer
     */
    addTrailPoint(pt) {
      const maxPts = this.options.maxTrailPoints;
      // Filter out points that haven't moved significantly
      if (this.trailPoints.length > 0) {
        const last = this.trailPoints[this.trailPoints.length - 1];
        if (pt.distanceTo(last) < 0.008) return;
      }

      this.trailPoints.push(pt.clone());
      if (this.trailPoints.length > maxPts) {
        this.trailPoints.shift();
      }

      // Update geometry buffer
      const posAttr = this.trailGeometry.attributes.position;
      const colAttr = this.trailGeometry.attributes.color;
      const len = this.trailPoints.length;

      const c1 = new THREE.Color(0x00f2fe); // Cyan lead
      const c2 = new THREE.Color(0x9b51e0); // Purple tail
      const tmpColor = new THREE.Color();

      for (let i = 0; i < maxPts; i++) {
        if (i < len) {
          const p = this.trailPoints[i];
          posAttr.setXYZ(i, p.x, p.y, p.z);
          const t = i / Math.max(1, len - 1);
          tmpColor.copy(c2).lerp(c1, t);
          colAttr.setXYZ(i, tmpColor.r, tmpColor.g, tmpColor.b);
        } else if (len > 0) {
          const last = this.trailPoints[len - 1];
          posAttr.setXYZ(i, last.x, last.y, last.z);
          colAttr.setXYZ(i, c1.r, c1.g, c1.b);
        }
      }

      posAttr.needsUpdate = true;
      colAttr.needsUpdate = true;
      this.trailGeometry.setDrawRange(0, len);
    }

    /**
     * Clear all recorded trajectory trail points
     */
    clearTrajectory() {
      this.trailPoints = [];
      this.trailGeometry.setDrawRange(0, 0);
      this.trailGeometry.attributes.position.needsUpdate = true;
    }

    /**
     * Set Theme Palette
     */
    setTheme(themeName) {
      this.options.theme = themeName;
      const armColors = {
        orange: 0xff6b08,
        yellow: 0xf1c40f,
        white: 0xecf0f1,
        cyan: 0x00d2d3
      };
      const col = armColors[themeName] || 0xff6b08;
      if (this.materials.robotArm) {
        this.materials.robotArm.color.setHex(col);
      }
    }

    /**
     * Set Camera Preset View
     */
    setCameraView(viewName) {
      const target = new THREE.Vector3(0, 0, 1.2);
      this.controls.target.copy(target);

      switch (viewName) {
        case 'isometric':
          this.camera.position.set(3.8, -4.5, 3.2);
          break;
        case 'top':
          this.camera.position.set(0.01, 0, 5.5);
          break;
        case 'front':
          this.camera.position.set(0, -5.2, 1.5);
          break;
        case 'side':
          this.camera.position.set(5.2, 0, 1.5);
          break;
        case 'targetA':
          this.camera.position.set(2.4, 2.4, 1.6);
          this.controls.target.set(1.0, 1.0, 0.4);
          break;
        case 'targetB':
          this.camera.position.set(-2.4, 2.6, 1.6);
          this.controls.target.set(-1.0, 1.2, 0.4);
          break;
      }
      this.controls.update();
    }

    bindEvents() {
      window.addEventListener('resize', () => {
        const width = this.container.clientWidth;
        const height = this.container.clientHeight;
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
      });
    }

    render() {
      requestAnimationFrame(this.animate);
    }

    animate() {
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
      requestAnimationFrame(this.animate);
    }
  }

  // SceneUtils detach helper for Three.js
  if (typeof THREE !== 'undefined' && !THREE.SceneUtils) {
    THREE.SceneUtils = {
      detach: function (child, parent, scene) {
        child.applyMatrix4(parent.matrixWorld);
        parent.remove(child);
        scene.add(child);
      }
    };
  }

  global.RobotVisualizer = RobotVisualizer;

})(typeof window !== 'undefined' ? window : this);
