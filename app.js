/**
 * app.js
 * 
 * Application Controller & Orchestration Engine for 3-DOF Robot Simulation.
 * Connects Kinematics, Three.js 3D Visualizer, UI Controls, Telemetry Dashboard,
 * Web Audio Sound Synthesizer, and Pick-and-Place Automation State Machine.
 */

(function () {
  'use strict';

  // Audio Synthesizer for Realistic Industrial Robot Sounds (Web Audio API)
  class RobotAudioSystem {
    constructor() {
      this.enabled = true;
      this.audioCtx = null;
      this.servoOsc = null;
      this.servoGain = null;
      this.isServoRunning = false;
    }

    init() {
      if (this.audioCtx) return;
      try {
        const AudioContext = window.AudioContext || window.webkitAudioContext;
        this.audioCtx = new AudioContext();
      } catch (e) {
        console.warn('Web Audio API not supported on this browser.');
      }
    }

    resume() {
      if (this.audioCtx && this.audioCtx.state === 'suspended') {
        this.audioCtx.resume();
      }
    }

    startServoHum(speed = 1.0) {
      if (!this.enabled) return;
      this.init();
      this.resume();
      if (!this.audioCtx) return;

      if (!this.isServoRunning) {
        this.servoOsc = this.audioCtx.createOscillator();
        this.servoGain = this.audioCtx.createGain();

        this.servoOsc.type = 'sawtooth';
        // Base frequency for industrial servomotor
        this.servoOsc.frequency.setValueAtTime(120, this.audioCtx.currentTime);

        // Low-pass filter to make sound warm and mechanical
        const filter = this.audioCtx.createBiquadFilter();
        filter.type = 'lowpass';
        filter.frequency.setValueAtTime(450, this.audioCtx.currentTime);

        this.servoGain.gain.setValueAtTime(0.001, this.audioCtx.currentTime);
        this.servoGain.gain.linearRampToValueAtTime(0.035, this.audioCtx.currentTime + 0.15);

        this.servoOsc.connect(filter);
        filter.connect(this.servoGain);
        this.servoGain.connect(this.audioCtx.destination);

        this.servoOsc.start();
        this.isServoRunning = true;
      }
    }

    updateServoPitch(jointVelocity) {
      if (!this.isServoRunning || !this.servoOsc || !this.audioCtx) return;
      const v = Math.min(3.0, Math.abs(jointVelocity));
      const targetFreq = 110 + v * 90;
      this.servoOsc.frequency.setTargetAtTime(targetFreq, this.audioCtx.currentTime, 0.05);
    }

    stopServoHum() {
      if (this.isServoRunning && this.servoGain && this.audioCtx) {
        this.servoGain.gain.linearRampToValueAtTime(0.0001, this.audioCtx.currentTime + 0.12);
        setTimeout(() => {
          if (this.servoOsc) {
            try { this.servoOsc.stop(); } catch (e) {}
            this.servoOsc.disconnect();
            this.servoOsc = null;
          }
          this.isServoRunning = false;
        }, 130);
      }
    }

    playPneumaticGrasp() {
      if (!this.enabled) return;
      this.init();
      this.resume();
      if (!this.audioCtx) return;

      // Burst of filtered white noise (pneumatic hiss)
      const bufferSize = this.audioCtx.sampleRate * 0.15;
      const noiseBuffer = this.audioCtx.createBuffer(1, bufferSize, this.audioCtx.sampleRate);
      const output = noiseBuffer.getChannelData(0);
      for (let i = 0; i < bufferSize; i++) {
        output[i] = Math.random() * 2 - 1;
      }

      const whiteNoise = this.audioCtx.createBufferSource();
      whiteNoise.buffer = noiseBuffer;

      const filter = this.audioCtx.createBiquadFilter();
      filter.type = 'bandpass';
      filter.frequency.value = 1400;
      filter.Q.value = 2.0;

      const noiseGain = this.audioCtx.createGain();
      noiseGain.gain.setValueAtTime(0.08, this.audioCtx.currentTime);
      noiseGain.gain.exponentialRampToValueAtTime(0.001, this.audioCtx.currentTime + 0.14);

      whiteNoise.connect(filter);
      filter.connect(noiseGain);
      noiseGain.connect(this.audioCtx.destination);
      whiteNoise.start();

      // Magnetic / Mechanical Click
      const osc = this.audioCtx.createOscillator();
      const clickGain = this.audioCtx.createGain();
      osc.type = 'triangle';
      osc.frequency.setValueAtTime(320, this.audioCtx.currentTime);
      osc.frequency.exponentialRampToValueAtTime(80, this.audioCtx.currentTime + 0.08);

      clickGain.gain.setValueAtTime(0.12, this.audioCtx.currentTime);
      clickGain.gain.exponentialRampToValueAtTime(0.001, this.audioCtx.currentTime + 0.08);

      osc.connect(clickGain);
      clickGain.connect(this.audioCtx.destination);
      osc.start();
      osc.stop(this.audioCtx.currentTime + 0.09);
    }

    playDropChime() {
      if (!this.enabled) return;
      this.init();
      this.resume();
      if (!this.audioCtx) return;

      const osc = this.audioCtx.createOscillator();
      const gain = this.audioCtx.createGain();
      osc.type = 'sine';
      osc.frequency.setValueAtTime(587.33, this.audioCtx.currentTime); // D5
      osc.frequency.setValueAtTime(880, this.audioCtx.currentTime + 0.08); // A5

      gain.gain.setValueAtTime(0.09, this.audioCtx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, this.audioCtx.currentTime + 0.35);

      osc.connect(gain);
      gain.connect(this.audioCtx.destination);
      osc.start();
      osc.stop(this.audioCtx.currentTime + 0.36);
    }
  }

  // Automation Sequence Steps
  const AutomationSteps = Object.freeze({
    IDLE: 'IDLE',
    START_TO_APPROACH_A: 'START_TO_APPROACH_A',
    DESCEND_TO_PICK: 'DESCEND_TO_PICK',
    PICK_GRASP: 'PICK_GRASP',
    LIFT_FROM_A: 'LIFT_FROM_A',
    TRANSFER_TO_B: 'TRANSFER_TO_B',
    DESCEND_TO_PLACE: 'DESCEND_TO_PLACE',
    PLACE_RELEASE: 'PLACE_RELEASE',
    RETRACT_FROM_B: 'RETRACT_FROM_B',
    RETURN_HOME: 'RETURN_HOME',
    CYCLE_COMPLETE: 'CYCLE_COMPLETE'
  });

  class AppController {
    constructor() {
      this.visualizer = new RobotVisualizer('canvas-container');
      this.audio = new RobotAudioSystem();

      // Joint states in radians
      this.joints = {
        theta1: Kinematics.ROBOT_PARAMS.home.theta1,
        theta2: Kinematics.ROBOT_PARAMS.home.theta2,
        theta3: Kinematics.ROBOT_PARAMS.home.theta3
      };

      // Automation State Machine
      this.autoState = AutomationSteps.IDLE;
      this.autoTimer = 0;
      this.isPaused = false;
      this.speedMultiplier = 1.0;
      this.continuousLoop = false;
      this.trajectoryType = 'cubic'; // 'cubic' or 'quintic'

      // Active Trajectory Motion
      this.activeTrajectory = null;
      this.trajElapsed = 0;
      this.onTrajComplete = null;

      // Waypoint definitions
      this.initWaypoints();

      // UI Element references
      this.bindElements();
      this.attachEventListeners();

      // Initial telemetry and UI sync
      this.updateJoints(this.joints.theta1, this.joints.theta2, this.joints.theta3, true);
      this.updateTelemetry();

      // Animation Tick Loop
      this.lastTimestamp = performance.now();
      this.loop = this.loop.bind(this);
      requestAnimationFrame(this.loop);
    }

    initWaypoints() {
      // Home Joint Configuration: [0°, 45°, -45°]
      this.homeJoints = {
        theta1: Kinematics.ROBOT_PARAMS.home.theta1,
        theta2: Kinematics.ROBOT_PARAMS.home.theta2,
        theta3: Kinematics.ROBOT_PARAMS.home.theta3
      };

      // Target A: [1.0, 1.0, 0.2]
      this.posTargetA = { x: 1.0, y: 1.0, z: 0.2 };
      this.posApproachA = { x: 1.0, y: 1.0, z: 0.55 };

      // Target B: [-1.0, 1.2, 0.2]
      this.posTargetB = { x: -1.0, y: 1.2, z: 0.2 };
      this.posApproachB = { x: -1.0, y: 1.2, z: 0.55 };

      // Solve IK for all waypoints
      this.ikPickA = Kinematics.inverseKinematics(this.posTargetA.x, this.posTargetA.y, this.posTargetA.z);
      this.ikApproachA = Kinematics.inverseKinematics(this.posApproachA.x, this.posApproachA.y, this.posApproachA.z);
      this.ikPlaceB = Kinematics.inverseKinematics(this.posTargetB.x, this.posTargetB.y, this.posTargetB.z);
      this.ikApproachB = Kinematics.inverseKinematics(this.posApproachB.x, this.posApproachB.y, this.posApproachB.z);
    }

    bindElements() {
      // Sliders & Number Badges
      this.slider1 = document.getElementById('slider-t1');
      this.slider2 = document.getElementById('slider-t2');
      this.slider3 = document.getElementById('slider-t3');

      this.valT1Deg = document.getElementById('val-t1-deg');
      this.valT2Deg = document.getElementById('val-t2-deg');
      this.valT3Deg = document.getElementById('val-t3-deg');

      this.valT1Rad = document.getElementById('val-t1-rad');
      this.valT2Rad = document.getElementById('val-t2-rad');
      this.valT3Rad = document.getElementById('val-t3-rad');

      // IK Inputs & Buttons
      this.inputX = document.getElementById('ik-input-x');
      this.inputY = document.getElementById('ik-input-y');
      this.inputZ = document.getElementById('ik-input-z');
      this.btnSolveIK = document.getElementById('btn-solve-ik');
      this.btnFillTargetA = document.getElementById('btn-preset-target-a');
      this.btnFillTargetB = document.getElementById('btn-preset-target-b');
      this.selectElbowConfig = document.getElementById('select-elbow-config');

      // Automation Action Buttons
      this.btnStartAuto = document.getElementById('btn-start-auto');
      this.btnPauseAuto = document.getElementById('btn-pause-auto');
      this.btnResetAuto = document.getElementById('btn-reset-auto');
      this.btnHomePose = document.getElementById('btn-home-pose');

      // Trajectory & Motion Controls
      this.speedRange = document.getElementById('speed-range');
      this.speedLabel = document.getElementById('speed-label');
      this.chkContinuous = document.getElementById('chk-continuous');
      this.selectTrajectoryType = document.getElementById('select-traj-type');
      this.chkTraceTrail = document.getElementById('chk-trace-trail');
      this.btnClearTrail = document.getElementById('btn-clear-trail');

      // Telemetry Displays
      this.telemX = document.getElementById('telem-x');
      this.telemY = document.getElementById('telem-y');
      this.telemZ = document.getElementById('telem-z');
      this.telemReach = document.getElementById('telem-reach');
      this.telemReachBar = document.getElementById('telem-reach-bar');
      this.statusBadge = document.getElementById('status-badge');
      this.stepIndicator = document.getElementById('step-indicator');
      this.singularityWarning = document.getElementById('singularity-warning');

      // DH Matrix Output Elements
      this.dhCells = [];
      for (let r = 0; r < 4; r++) {
        this.dhCells[r] = [];
        for (let c = 0; c < 4; c++) {
          this.dhCells[r][c] = document.getElementById(`dh-m-${r}-${c}`);
        }
      }

      // Audio Toggle Button
      this.btnToggleAudio = document.getElementById('btn-toggle-audio');
    }

    attachEventListeners() {
      // Manual Joint Slider Inputs (Forward Kinematics)
      const onSliderChange = () => {
        if (this.autoState !== AutomationSteps.IDLE) {
          this.stopAutomation();
        }
        const t1 = Kinematics.degToRad(parseFloat(this.slider1.value));
        const t2 = Kinematics.degToRad(parseFloat(this.slider2.value));
        const t3 = Kinematics.degToRad(parseFloat(this.slider3.value));
        this.updateJoints(t1, t2, t3, false);
      };

      this.slider1.addEventListener('input', onSliderChange);
      this.slider2.addEventListener('input', onSliderChange);
      this.slider3.addEventListener('input', onSliderChange);

      // Solve IK & Move Button
      this.btnSolveIK.addEventListener('click', () => {
        const x = parseFloat(this.inputX.value);
        const y = parseFloat(this.inputY.value);
        const z = parseFloat(this.inputZ.value);
        this.executeMoveToXYZ(x, y, z);
      });

      // Quick-fill Target Presets
      this.btnFillTargetA.addEventListener('click', () => {
        this.inputX.value = this.posTargetA.x.toFixed(2);
        this.inputY.value = this.posTargetA.y.toFixed(2);
        this.inputZ.value = this.posTargetA.z.toFixed(2);
        this.executeMoveToXYZ(this.posTargetA.x, this.posTargetA.y, this.posTargetA.z);
      });

      this.btnFillTargetB.addEventListener('click', () => {
        this.inputX.value = this.posTargetB.x.toFixed(2);
        this.inputY.value = this.posTargetB.y.toFixed(2);
        this.inputZ.value = this.posTargetB.z.toFixed(2);
        this.executeMoveToXYZ(this.posTargetB.x, this.posTargetB.y, this.posTargetB.z);
      });

      // Home Position Button
      this.btnHomePose.addEventListener('click', () => {
        this.stopAutomation();
        this.moveToJointTarget(this.homeJoints, 1.2, () => {
          this.setStatus('AT HOME', 'idle');
        });
      });

      // Start Pick-and-Place Automation
      this.btnStartAuto.addEventListener('click', () => {
        if (this.isPaused) {
          this.resumeAutomation();
        } else {
          this.startPickAndPlaceAutomation();
        }
      });

      // Pause Automation
      this.btnPauseAuto.addEventListener('click', () => {
        this.pauseAutomation();
      });

      // Reset Automation & Scene
      this.btnResetAuto.addEventListener('click', () => {
        this.resetAutomation();
      });

      // Speed Slider
      this.speedRange.addEventListener('input', (e) => {
        this.speedMultiplier = parseFloat(e.target.value);
        this.speedLabel.textContent = `${this.speedMultiplier.toFixed(2)}x`;
      });

      // Continuous Loop Toggle
      this.chkContinuous.addEventListener('change', (e) => {
        this.continuousLoop = e.target.checked;
      });

      // Trajectory Type Selector
      this.selectTrajectoryType.addEventListener('change', (e) => {
        this.trajectoryType = e.target.value;
      });

      // Trajectory Trail Toggle & Clear
      this.chkTraceTrail.addEventListener('change', (e) => {
        this.visualizer.options.traceTrail = e.target.checked;
        if (!e.target.checked) {
          this.visualizer.clearTrajectory();
        }
      });

      this.btnClearTrail.addEventListener('click', () => {
        this.visualizer.clearTrajectory();
      });

      // Audio Toggle Button
      this.btnToggleAudio.addEventListener('click', () => {
        this.audio.enabled = !this.audio.enabled;
        this.btnToggleAudio.classList.toggle('active', this.audio.enabled);
        this.btnToggleAudio.innerHTML = this.audio.enabled ? '🔊 Audio ON' : '🔇 Audio Muted';
      });

      // Camera Presets Buttons
      document.querySelectorAll('[data-cam-view]').forEach(btn => {
        btn.addEventListener('click', (e) => {
          const view = e.currentTarget.getAttribute('data-cam-view');
          this.visualizer.setCameraView(view);
          document.querySelectorAll('[data-cam-view]').forEach(b => b.classList.remove('active'));
          e.currentTarget.classList.add('active');
        });
      });

      // Theme Preset Buttons
      document.querySelectorAll('[data-theme]').forEach(btn => {
        btn.addEventListener('click', (e) => {
          const th = e.currentTarget.getAttribute('data-theme');
          this.visualizer.setTheme(th);
          document.querySelectorAll('[data-theme]').forEach(b => b.classList.remove('active'));
          e.currentTarget.classList.add('active');
        });
      });

      // Keyboard Shortcuts
      window.addEventListener('keydown', (e) => {
        if (e.target.tagName === 'INPUT') return; // Don't intercept text inputs
        if (e.code === 'Space') {
          e.preventDefault();
          if (this.autoState === AutomationSteps.IDLE || this.autoState === AutomationSteps.CYCLE_COMPLETE) {
            this.startPickAndPlaceAutomation();
          } else {
            this.pauseAutomation();
          }
        } else if (e.key === 'h' || e.key === 'H') {
          this.btnHomePose.click();
        } else if (e.key === 'r' || e.key === 'R') {
          this.resetAutomation();
        } else if (e.key === '1') {
          this.visualizer.setCameraView('isometric');
        } else if (e.key === '2') {
          this.visualizer.setCameraView('top');
        } else if (e.key === '3') {
          this.visualizer.setCameraView('front');
        } else if (e.key === '4') {
          this.visualizer.setCameraView('side');
        }
      });
    }

    /**
     * Solve IK and execute smooth trajectory to (X, Y, Z)
     */
    executeMoveToXYZ(x, y, z) {
      this.stopAutomation();
      const elbowUp = this.selectElbowConfig.value === 'elbow_up';
      const ik = Kinematics.inverseKinematics(x, y, z, {
        elbowUp,
        currentTheta1: this.joints.theta1
      });

      if (!ik.success) {
        this.setStatus('IK FAILED', 'error');
        alert(ik.statusMessage);
        return;
      }

      // Display warning if clamped or singularity
      if (ik.clamped) {
        this.showWarning(`⚠️ ${ik.statusMessage}`);
      } else if (ik.singularity) {
        this.showWarning(`⚠️ Singularity detected: ${ik.singularity}`);
      } else {
        this.hideWarning();
      }

      this.setStatus('MOVING (IK)', 'running');
      const goal = { theta1: ik.theta1, theta2: ik.theta2, theta3: ik.theta3 };
      this.moveToJointTarget(goal, 1.4, () => {
        this.setStatus('REACHED', 'idle');
      });
    }

    /**
     * Start smooth joint trajectory interpolation
     */
    moveToJointTarget(qGoal, baseDuration, callback) {
      const maxDelta = Math.max(
        Math.abs(qGoal.theta1 - this.joints.theta1),
        Math.abs(qGoal.theta2 - this.joints.theta2),
        Math.abs(qGoal.theta3 - this.joints.theta3)
      );

      // Scale duration based on angular distance and speed multiplier
      const dur = Math.max(0.4, (baseDuration || 1.2) * (maxDelta / 1.2)) / this.speedMultiplier;

      if (this.trajectoryType === 'quintic') {
        this.activeTrajectory = new Kinematics.QuinticTrajectory(this.joints, qGoal, dur);
      } else {
        this.activeTrajectory = new Kinematics.CubicTrajectory(this.joints, qGoal, dur);
      }

      this.trajElapsed = 0;
      this.onTrajComplete = callback || null;

      // Start servo motor audio
      this.audio.startServoHum(this.speedMultiplier);
    }

    /**
     * Update Joint Rotations and Synchronize UI Controls
     */
    updateJoints(t1, t2, t3, updateSliders = false) {
      this.joints.theta1 = t1;
      this.joints.theta2 = t2;
      this.joints.theta3 = t3;

      // Update 3D Robot Visualizer
      this.visualizer.setJointAngles(t1, t2, t3);

      // Convert to Degrees
      const deg1 = Kinematics.radToDeg(t1);
      const deg2 = Kinematics.radToDeg(t2);
      const deg3 = Kinematics.radToDeg(t3);

      // Update Slider Elements if requested
      if (updateSliders) {
        this.slider1.value = deg1.toFixed(1);
        this.slider2.value = deg2.toFixed(1);
        this.slider3.value = deg3.toFixed(1);
      }

      // Update Numerical Labels
      this.valT1Deg.textContent = `${deg1 >= 0 ? '+' : ''}${deg1.toFixed(1)}°`;
      this.valT2Deg.textContent = `${deg2 >= 0 ? '+' : ''}${deg2.toFixed(1)}°`;
      this.valT3Deg.textContent = `${deg3 >= 0 ? '+' : ''}${deg3.toFixed(1)}°`;

      this.valT1Rad.textContent = `${t1.toFixed(3)} rad`;
      this.valT2Rad.textContent = `${t2.toFixed(3)} rad`;
      this.valT3Rad.textContent = `${t3.toFixed(3)} rad`;

      // Update Telemetry & DH Table
      this.updateTelemetry();
    }

    /**
     * Update Real-time Telemetry Dashboard and 4x4 DH Matrix Table
     */
    updateTelemetry() {
      const fk = Kinematics.forwardKinematics(this.joints.theta1, this.joints.theta2, this.joints.theta3);
      const pos = fk.position;

      this.telemX.textContent = `${pos.x.toFixed(3)} m`;
      this.telemY.textContent = `${pos.y.toFixed(3)} m`;
      this.telemZ.textContent = `${pos.z.toFixed(3)} m`;

      // Distance from shoulder
      const rShoulder = Math.sqrt(pos.x * pos.x + pos.y * pos.y);
      const zShoulder = pos.z - Kinematics.ROBOT_PARAMS.d1;
      const dShoulder = Math.sqrt(rShoulder * rShoulder + zShoulder * zShoulder);
      const reachPct = (dShoulder / Kinematics.ROBOT_PARAMS.dMax) * 100;

      this.telemReach.textContent = `${dShoulder.toFixed(3)}m (${reachPct.toFixed(0)}%)`;
      this.telemReachBar.style.width = `${Math.min(100, reachPct)}%`;
      if (reachPct > 98) {
        this.telemReachBar.style.backgroundColor = '#ef4444'; // Red limit
      } else if (reachPct > 80) {
        this.telemReachBar.style.backgroundColor = '#f59e0b'; // Amber
      } else {
        this.telemReachBar.style.backgroundColor = '#10b981'; // Green
      }

      // Update 4x4 DH Transformation Matrix Table (T03)
      const T03 = fk.matrices.T03;
      for (let r = 0; r < 4; r++) {
        for (let c = 0; c < 4; c++) {
          const val = T03[r][c];
          if (this.dhCells[r][c]) {
            this.dhCells[r][c].textContent = val.toFixed(3);
          }
        }
      }
    }

    /**
     * Set Status Badge State
     */
    setStatus(text, type = 'idle') {
      this.statusBadge.textContent = text;
      this.statusBadge.className = `status-badge status-${type}`;
    }

    /**
     * Set Automation Step Indicator
     */
    setStep(stepText) {
      if (this.stepIndicator) {
        this.stepIndicator.textContent = stepText;
      }
    }

    showWarning(msg) {
      if (this.singularityWarning) {
        this.singularityWarning.textContent = msg;
        this.singularityWarning.style.display = 'block';
      }
    }

    hideWarning() {
      if (this.singularityWarning) {
        this.singularityWarning.style.display = 'none';
      }
    }

    // =======================================================
    // PICK-AND-PLACE AUTOMATION STATE MACHINE
    // =======================================================

    startPickAndPlaceAutomation() {
      this.isPaused = false;
      this.visualizer.resetCubeToTargetA();
      this.visualizer.setGripper(1.0); // Open gripper
      this.hideWarning();

      this.btnStartAuto.classList.add('active');
      this.btnPauseAuto.classList.remove('active');
      this.setStatus('RUNNING', 'running');

      // Step 1: Arm starts at Home position [θ1=0°, θ2=45°, θ3=-45°]
      this.setStep('Step 1/5: Moving to Home Standby Pose');
      this.moveToJointTarget(this.homeJoints, 1.0, () => {
        this.transitionToStep(AutomationSteps.START_TO_APPROACH_A);
      });
    }

    pauseAutomation() {
      if (this.autoState === AutomationSteps.IDLE) return;
      this.isPaused = !this.isPaused;
      if (this.isPaused) {
        this.setStatus('PAUSED', 'paused');
        this.btnPauseAuto.classList.add('active');
        this.audio.stopServoHum();
      } else {
        this.setStatus('RUNNING', 'running');
        this.btnPauseAuto.classList.remove('active');
      }
    }

    resumeAutomation() {
      this.isPaused = false;
      this.setStatus('RUNNING', 'running');
      this.btnPauseAuto.classList.remove('active');
    }

    stopAutomation() {
      this.autoState = AutomationSteps.IDLE;
      this.activeTrajectory = null;
      this.isPaused = false;
      this.audio.stopServoHum();
      this.btnStartAuto.classList.remove('active');
      this.btnPauseAuto.classList.remove('active');
      this.setStep('Standby / Manual Control');
    }

    resetAutomation() {
      this.stopAutomation();
      this.visualizer.resetCubeToTargetA();
      this.visualizer.clearTrajectory();
      this.visualizer.setGripper(1.0);
      this.moveToJointTarget(this.homeJoints, 0.8, () => {
        this.updateJoints(this.homeJoints.theta1, this.homeJoints.theta2, this.homeJoints.theta3, true);
        this.setStatus('IDLE', 'idle');
        this.setStep('System Ready');
      });
    }

    transitionToStep(nextStep) {
      if (this.isPaused) {
        this.autoState = nextStep;
        return;
      }

      this.autoState = nextStep;

      switch (nextStep) {
        // Step 2: Moves smoothly via cubic joint trajectory to Target A
        case AutomationSteps.START_TO_APPROACH_A:
          this.setStep('Step 2/5: Approach Target A (Pick Point)');
          // Waypoint: Approach pose above Target A
          this.moveToJointTarget({
            theta1: this.ikApproachA.theta1,
            theta2: this.ikApproachA.theta2,
            theta3: this.ikApproachA.theta3
          }, 1.6, () => {
            this.transitionToStep(AutomationSteps.DESCEND_TO_PICK);
          });
          break;

        case AutomationSteps.DESCEND_TO_PICK:
          this.setStep('Step 2/5: Descending to Pick Position');
          // Lower end-effector to Target A pick position [1.0, 1.0, 0.2]
          this.moveToJointTarget({
            theta1: this.ikPickA.theta1,
            theta2: this.ikPickA.theta2,
            theta3: this.ikPickA.theta3
          }, 0.9, () => {
            this.transitionToStep(AutomationSteps.PICK_GRASP);
          });
          break;

        // Step 3: Lower end-effector, "Pick" the cube (attach cube mesh to end-effector tip)
        case AutomationSteps.PICK_GRASP:
          this.setStep('Step 3/5: Grasping Cube at Target A');
          this.audio.stopServoHum();
          this.audio.playPneumaticGrasp();
          
          // Animate gripper closing onto cube
          this.animateGripper(1.0, 0.0, 450, () => {
            this.visualizer.attachCubeToGripper();
            setTimeout(() => {
              this.transitionToStep(AutomationSteps.LIFT_FROM_A);
            }, 250);
          });
          break;

        // Step 4: Retract upward and move along trajectory to Target B
        case AutomationSteps.LIFT_FROM_A:
          this.setStep('Step 4/5: Retracting Upward from Pick');
          this.moveToJointTarget({
            theta1: this.ikApproachA.theta1,
            theta2: this.ikApproachA.theta2,
            theta3: this.ikApproachA.theta3
          }, 0.9, () => {
            this.transitionToStep(AutomationSteps.TRANSFER_TO_B);
          });
          break;

        case AutomationSteps.TRANSFER_TO_B:
          this.setStep('Step 4/5: Trajectory Interpolation to Target B');
          this.moveToJointTarget({
            theta1: this.ikApproachB.theta1,
            theta2: this.ikApproachB.theta2,
            theta3: this.ikApproachB.theta3
          }, 2.2, () => {
            this.transitionToStep(AutomationSteps.DESCEND_TO_PLACE);
          });
          break;

        case AutomationSteps.DESCEND_TO_PLACE:
          this.setStep('Step 5/5: Lowering Arm into Target B Box');
          this.moveToJointTarget({
            theta1: this.ikPlaceB.theta1,
            theta2: this.ikPlaceB.theta2,
            theta3: this.ikPlaceB.theta3
          }, 1.1, () => {
            this.transitionToStep(AutomationSteps.PLACE_RELEASE);
          });
          break;

        // Step 5: Lower arm and "Place" the cube inside Target B box, then return to Home
        case AutomationSteps.PLACE_RELEASE:
          this.setStep('Step 5/5: Releasing Cube into Target B Box');
          this.audio.stopServoHum();
          this.audio.playDropChime();

          // Detach cube and drop into box
          this.animateGripper(0.0, 1.0, 450, () => {
            this.visualizer.detachCubeToWorld(new THREE.Vector3(
              this.posTargetB.x,
              this.posTargetB.y,
              0.16
            ));
            setTimeout(() => {
              this.transitionToStep(AutomationSteps.RETRACT_FROM_B);
            }, 250);
          });
          break;

        case AutomationSteps.RETRACT_FROM_B:
          this.setStep('Retracting Upward from Target B');
          this.moveToJointTarget({
            theta1: this.ikApproachB.theta1,
            theta2: this.ikApproachB.theta2,
            theta3: this.ikApproachB.theta3
          }, 0.9, () => {
            this.transitionToStep(AutomationSteps.RETURN_HOME);
          });
          break;

        case AutomationSteps.RETURN_HOME:
          this.setStep('Returning to Home Position [0°, 45°, -45°]');
          this.moveToJointTarget(this.homeJoints, 1.6, () => {
            this.transitionToStep(AutomationSteps.CYCLE_COMPLETE);
          });
          break;

        case AutomationSteps.CYCLE_COMPLETE:
          this.setStep('Pick-and-Place Cycle Finished Successfully!');
          this.setStatus('CYCLE COMPLETE', 'idle');
          this.audio.stopServoHum();
          this.btnStartAuto.classList.remove('active');

          if (this.continuousLoop) {
            setTimeout(() => {
              if (this.continuousLoop) {
                this.startPickAndPlaceAutomation();
              }
            }, 1200);
          }
          break;
      }
    }

    /**
     * Animate gripper jaw opening / closing over duration
     */
    animateGripper(fromRatio, toRatio, durationMs, onDone) {
      const startT = performance.now();
      const animate = (now) => {
        const elapsed = (now - startT) / (durationMs / this.speedMultiplier);
        const t = Math.min(1.0, elapsed);
        const currentRatio = fromRatio + (toRatio - fromRatio) * t;
        this.visualizer.setGripper(currentRatio);
        if (t < 1.0) {
          requestAnimationFrame(animate);
        } else {
          if (onDone) onDone();
        }
      };
      requestAnimationFrame(animate);
    }

    // =======================================================
    // MAIN ANIMATION TICK LOOP
    // =======================================================

    loop(timestamp) {
      const dt = Math.min(0.1, (timestamp - this.lastTimestamp) / 1000);
      this.lastTimestamp = timestamp;

      // Handle active trajectory interpolation
      if (this.activeTrajectory && !this.isPaused) {
        this.trajElapsed += dt;
        const evalResult = this.activeTrajectory.evaluate(this.trajElapsed);

        // Update joint angles from trajectory evaluation
        this.updateJoints(evalResult.q.theta1, evalResult.q.theta2, evalResult.q.theta3, true);

        // Modulate servo audio frequency with angular velocity
        const maxV = Math.max(Math.abs(evalResult.v.theta1), Math.abs(evalResult.v.theta2), Math.abs(evalResult.v.theta3));
        this.audio.updateServoPitch(maxV);

        if (evalResult.done) {
          const cb = this.onTrajComplete;
          this.activeTrajectory = null;
          this.onTrajComplete = null;
          this.audio.stopServoHum();
          if (cb) cb();
        }
      }

      requestAnimationFrame(this.loop);
    }
  }

  // Initialize application on DOM ready
  window.addEventListener('DOMContentLoaded', () => {
    window.app = new AppController();
  });

})();
