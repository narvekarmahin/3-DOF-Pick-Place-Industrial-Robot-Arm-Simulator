/**
 * kinematics.js
 * 
 * High-Precision Kinematics & Trajectory Engine for 3-DOF Spatial Articulated Robot Arm (3-RRR).
 * 
 * Kinematic Specifications:
 * - Base Joint 1: Revolute around Z-axis (θ1)
 * - Shoulder Joint 2: Revolute around local Y-axis (θ2)
 * - Elbow Joint 3: Revolute around local Y-axis (θ3)
 * - Link 1 (Height / Base Offset): d1 = 1.5 m
 * - Link 2 (Upper Arm Length): a2 = 1.2 m
 * - Link 3 (Forearm Length):   a3 = 0.8 m
 * 
 * Denavit-Hartenberg (DH) Convention:
 * Joint 1: θ1 = variable, d1 = 1.5 m, a1 = 0.0 m, α1 = +90° (+π/2 rad)
 * Joint 2: θ2 = variable, d2 = 0.0 m, a2 = 1.2 m, α2 = 0.0° (0 rad)
 * Joint 3: θ3 = variable, d3 = 0.0 m, a3 = 0.8 m, α3 = 0.0° (0 rad)
 */

(function (global) {
  'use strict';

  // Constant Robot Physical Parameters
  const ROBOT_PARAMS = Object.freeze({
    d1: 1.5,      // Link 1 height (m)
    a2: 1.2,      // Link 2 length (m)
    a3: 0.8,      // Link 3 length (m)
    dMax: 2.0,    // a2 + a3 maximum physical reach from shoulder (m)
    dMin: 0.4,    // |a2 - a3| minimum physical reach from shoulder (m)
    limits: {
      theta1: { min: -Math.PI, max: Math.PI },          // -180° to +180°
      theta2: { min: -Math.PI / 2, max: Math.PI / 2 },  // -90° to +90°
      theta3: { min: -3 * Math.PI / 4, max: 3 * Math.PI / 4 } // -135° to +135°
    },
    home: {
      theta1: 0,
      theta2: Math.PI / 4,       // 45°
      theta3: -Math.PI / 4       // -45°
    }
  });

  /**
   * Clamp a value between min and max
   */
  function clamp(val, min, max) {
    return Math.max(min, Math.min(max, val));
  }

  /**
   * Convert Radians to Degrees
   */
  function radToDeg(rad) {
    return rad * (180 / Math.PI);
  }

  /**
   * Convert Degrees to Radians
   */
  function degToRad(deg) {
    return deg * (Math.PI / 180);
  }

  /**
   * Compute a 4x4 Homogeneous Transformation Matrix using Standard DH Parameters:
   * 
   * T = [ cos(θ)  -sin(θ)*cos(α)   sin(θ)*sin(α)  a*cos(θ) ]
   *     [ sin(θ)   cos(θ)*cos(α)  -cos(θ)*sin(α)  a*sin(θ) ]
   *     [   0          sin(α)          cos(α)        d     ]
   *     [   0            0               0           1     ]
   * 
   * Represented as a 4x4 array of rows: M[row][col]
   */
  function getDHMatrix(theta, d, a, alpha) {
    const ct = Math.cos(theta);
    const st = Math.sin(theta);
    const ca = Math.cos(alpha);
    const sa = Math.sin(alpha);

    return [
      [ct, -st * ca,  st * sa, a * ct],
      [st,  ct * ca, -ct * sa, a * st],
      [ 0,       sa,       ca,      d],
      [ 0,        0,        0,      1]
    ];
  }

  /**
   * Multiply two 4x4 matrices A and B (A x B)
   */
  function multiply4x4(A, B) {
    const result = [
      [0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 0, 0]
    ];
    for (let r = 0; r < 4; r++) {
      for (let c = 0; c < 4; c++) {
        let sum = 0;
        for (let k = 0; k < 4; k++) {
          sum += A[r][k] * B[k][c];
        }
        result[r][c] = sum;
      }
    }
    return result;
  }

  /**
   * Transform a 3D point using a 4x4 homogeneous transformation matrix
   */
  function transformPoint(M, pt) {
    const x = pt.x || 0;
    const y = pt.y || 0;
    const z = pt.z || 0;
    return {
      x: M[0][0] * x + M[0][1] * y + M[0][2] * z + M[0][3],
      y: M[1][0] * x + M[1][1] * y + M[1][2] * z + M[1][3],
      z: M[2][0] * x + M[2][1] * y + M[2][2] * z + M[2][3]
    };
  }

  /**
   * Forward Kinematics (FK)
   * 
   * Computes the end-effector pose and intermediate joint locations given joint angles (θ1, θ2, θ3).
   * 
   * Closed-form analytical expressions:
   * θ23 = θ2 + θ3
   * r   = a2 * cos(θ2) + a3 * cos(θ23)
   * X   = r * cos(θ1)
   * Y   = r * sin(θ1)
   * Z   = d1 + a2 * sin(θ2) + a3 * sin(θ23)
   * 
   * @param {number} theta1 Joint 1 angle (rad)
   * @param {number} theta2 Joint 2 angle (rad)
   * @param {number} theta3 Joint 3 angle (rad)
   * @returns {Object} FK results including end-effector coordinates and full DH transformation matrices
   */
  function forwardKinematics(theta1, theta2, theta3) {
    const { d1, a2, a3 } = ROBOT_PARAMS;

    // Direct closed-form trigonometric calculation
    const t23 = theta2 + theta3;
    const r = a2 * Math.cos(theta2) + a3 * Math.cos(t23);
    const eeX = r * Math.cos(theta1);
    const eeY = r * Math.sin(theta1);
    const eeZ = d1 + a2 * Math.sin(theta2) + a3 * Math.sin(t23);

    // Intermediate joint centers
    const p0 = { x: 0, y: 0, z: 0 };                        // Base Origin
    const p1 = { x: 0, y: 0, z: d1 };                       // Shoulder Joint (at top of Link 1)
    const p2 = {                                            // Elbow Joint (at end of Link 2)
      x: a2 * Math.cos(theta2) * Math.cos(theta1),
      y: a2 * Math.cos(theta2) * Math.sin(theta1),
      z: d1 + a2 * Math.sin(theta2)
    };
    const p3 = { x: eeX, y: eeY, z: eeZ };                  // End-Effector Tip (at end of Link 3)

    // DH Transformation Matrices
    const T01 = getDHMatrix(theta1, d1, 0, Math.PI / 2);
    const T12 = getDHMatrix(theta2, 0, a2, 0);
    const T23 = getDHMatrix(theta3, 0, a3, 0);

    const T02 = multiply4x4(T01, T12);
    const T03 = multiply4x4(T02, T23);

    return {
      position: { x: eeX, y: eeY, z: eeZ, r: Math.abs(r) },
      jointPositions: [p0, p1, p2, p3],
      matrices: {
        T01,
        T12,
        T23,
        T02,
        T03
      },
      angles: { theta1, theta2, theta3 }
    };
  }

  /**
   * Analytic Closed-Form Inverse Kinematics (IK)
   * 
   * Computes the joint angles (θ1, θ2, θ3) required to position the end-effector at (X, Y, Z).
   * 
   * Solves:
   * 1. θ1 = atan2(Y, X)
   * 2. Planar projection from shoulder: r = sqrt(X^2 + Y^2), z' = Z - d1
   * 3. Straight distance from shoulder D = sqrt(r^2 + z'^2)
   * 4. Reachability boundary check: D must be in [dMin, dMax]
   * 5. Law of Cosines for θ3: cos(θ3) = (D^2 - a2^2 - a3^2) / (2 * a2 * a3)
   * 6. Law of Cosines for θ2: θ2 = atan2(z', r) + gamma (elbow-down) or - gamma (elbow-up)
   * 
   * Gracefully handles:
   * - Workspace reach boundary exceedances (smooth projection to nearest reachable envelope point)
   * - Inner dead-zone singularity
   * - Shoulder axis singularity (when X=0, Y=0)
   * 
   * @param {number} targetX Target X coordinate (m)
   * @param {number} targetY Target Y coordinate (m)
   * @param {number} targetZ Target Z coordinate (m)
   * @param {Object} [options] Configuration options { elbowUp: boolean, currentTheta1: number }
   * @returns {Object} IK solution with status, joint angles, warnings, and reachability metrics
   */
  function inverseKinematics(targetX, targetY, targetZ, options = {}) {
    const { d1, a2, a3, dMax, dMin, limits } = ROBOT_PARAMS;
    const elbowUp = options.elbowUp || false;
    const currentTheta1 = options.currentTheta1 !== undefined ? options.currentTheta1 : 0;

    let x = Number(targetX);
    let y = Number(targetY);
    let z = Number(targetZ);

    let singularity = null;
    let clamped = false;
    let statusMessage = "IK solution exact.";

    // 1. Solve Joint 1 (Base Rotation θ1)
    let r = Math.sqrt(x * x + y * y);
    let theta1;

    // Check for shoulder singularity (on vertical Z-axis)
    if (r < 1e-5) {
      theta1 = currentTheta1;
      singularity = "SHOULDER_SINGULARITY";
      statusMessage = "Target is on the vertical base axis (Shoulder Singularity). θ1 preserved.";
    } else {
      theta1 = Math.atan2(y, x);
    }

    // 2. Planar Coordinates relative to Shoulder Joint at (0, 0, d1)
    let zp = z - d1;
    let D = Math.sqrt(r * r + zp * zp);
    const rawDistance = D;

    let clampedTarget = { x, y, z };

    // 3. Reachability Boundary Check & Graceful Handling
    const EPS = 0.0002;
    if (D > (dMax - EPS)) {
      // Outer reachability boundary exceedance
      clamped = true;
      const scale = (dMax - EPS) / D;
      r *= scale;
      zp *= scale;
      D = dMax - EPS;
      clampedTarget = {
        x: r * Math.cos(theta1),
        y: r * Math.sin(theta1),
        z: d1 + zp
      };
      singularity = "WORKSPACE_BOUNDARY_MAX";
      statusMessage = `Target exceeds max reach envelope (${rawDistance.toFixed(3)}m > ${dMax.toFixed(2)}m). Clamped to boundary.`;
    } else if (D < (dMin + EPS)) {
      // Inner dead-zone singularity
      clamped = true;
      const scale = (dMin + EPS) / (D > 1e-5 ? D : 1);
      r *= scale;
      zp *= scale;
      D = dMin + EPS;
      clampedTarget = {
        x: r * Math.cos(theta1),
        y: r * Math.sin(theta1),
        z: d1 + zp
      };
      singularity = "WORKSPACE_BOUNDARY_MIN";
      statusMessage = `Target inside inner dead-zone (${rawDistance.toFixed(3)}m < ${dMin.toFixed(2)}m). Clamped to boundary.`;
    }

    // 4. Solve Joint 3 (Elbow Rotation θ3) using Law of Cosines
    // D^2 = a2^2 + a3^2 + 2 * a2 * a3 * cos(θ3)
    let cosTheta3 = (D * D - a2 * a2 - a3 * a3) / (2 * a2 * a3);
    cosTheta3 = clamp(cosTheta3, -1.0, 1.0);

    let theta3;
    if (elbowUp) {
      theta3 = Math.acos(cosTheta3);
    } else {
      theta3 = -Math.acos(cosTheta3); // Default elbow-down (negative angle bends downward)
    }

    // 5. Solve Joint 2 (Shoulder Rotation θ2)
    // β = atan2(z', r)
    // γ = acos((a2^2 + D^2 - a3^2) / (2 * a2 * D))
    const beta = Math.atan2(zp, r);
    let cosGamma = (a2 * a2 + D * D - a3 * a3) / (2 * a2 * D);
    cosGamma = clamp(cosGamma, -1.0, 1.0);
    const gamma = Math.acos(cosGamma);

    let theta2;
    if (elbowUp) {
      theta2 = beta - gamma;
    } else {
      theta2 = beta + gamma; // Default elbow-down
    }

    // 6. Joint Limits Verification
    const withinLimits = (
      theta1 >= limits.theta1.min - 1e-4 && theta1 <= limits.theta1.max + 1e-4 &&
      theta2 >= limits.theta2.min - 1e-4 && theta2 <= limits.theta2.max + 1e-4 &&
      theta3 >= limits.theta3.min - 1e-4 && theta3 <= limits.theta3.max + 1e-4
    );

    if (!withinLimits) {
      statusMessage += " [Note: Joint angle near physical joint mechanical limit.]";
    }

    // Verify solution with Forward Kinematics
    const fkVerify = forwardKinematics(theta1, theta2, theta3);
    const posError = Math.sqrt(
      Math.pow(fkVerify.position.x - clampedTarget.x, 2) +
      Math.pow(fkVerify.position.y - clampedTarget.y, 2) +
      Math.pow(fkVerify.position.z - clampedTarget.z, 2)
    );

    return {
      success: true,
      theta1,
      theta2,
      theta3,
      anglesDeg: {
        theta1: radToDeg(theta1),
        theta2: radToDeg(theta2),
        theta3: radToDeg(theta3)
      },
      clamped,
      singularity,
      statusMessage,
      distanceFromShoulder: D,
      rawDistance,
      reachRatio: clamp(rawDistance / dMax, 0, 1.2),
      clampedTarget,
      positionError: posError,
      elbowUp
    };
  }

  /**
   * Cubic Polynomial Joint Trajectory Interpolator
   * 
   * q(t) = a0 + a1*t + a2*t^2 + a3*t^3
   * With zero boundary velocities: dq(0) = 0, dq(T) = 0
   * s(τ) = 3*τ^2 - 2*τ^3   where τ = t / T ∈ [0, 1]
   * 
   * Provides C1 continuity (continuous velocity profile).
   */
  class CubicTrajectory {
    constructor(qStart, qEnd, duration) {
      this.qStart = { ...qStart };
      this.qEnd = { ...qEnd };
      this.duration = Math.max(0.001, duration);
    }

    evaluate(t) {
      const tau = clamp(t / this.duration, 0, 1);
      // Smoothstep polynomial
      const s = 3 * tau * tau - 2 * tau * tau * tau;
      // Velocity factor: ds/dt = (6*tau - 6*tau^2) / T
      const ds = (6 * tau - 6 * tau * tau) / this.duration;
      // Acceleration factor: d2s/dt2 = (6 - 12*tau) / T^2
      const d2s = (6 - 12 * tau) / (this.duration * this.duration);

      const q = {};
      const v = {};
      const a = {};

      ['theta1', 'theta2', 'theta3'].forEach(joint => {
        const delta = this.qEnd[joint] - this.qStart[joint];
        q[joint] = this.qStart[joint] + delta * s;
        v[joint] = delta * ds;
        a[joint] = delta * d2s;
      });

      return { q, v, a, tau, done: tau >= 1.0 };
    }
  }

  /**
   * Quintic (5th-Order) Polynomial Minimum-Jerk Trajectory Interpolator
   * 
   * s(τ) = 10*τ^3 - 15*τ^4 + 6*τ^5
   * With zero boundary velocities AND zero boundary accelerations:
   * dq(0) = dq(T) = 0, d2q(0) = d2q(T) = 0
   * 
   * Provides C2 continuity (continuous acceleration, zero jerk at endpoints).
   */
  class QuinticTrajectory {
    constructor(qStart, qEnd, duration) {
      this.qStart = { ...qStart };
      this.qEnd = { ...qEnd };
      this.duration = Math.max(0.001, duration);
    }

    evaluate(t) {
      const tau = clamp(t / this.duration, 0, 1);
      const tau2 = tau * tau;
      const tau3 = tau2 * tau;
      const tau4 = tau3 * tau;
      const tau5 = tau4 * tau;

      // Minimum jerk polynomial
      const s = 10 * tau3 - 15 * tau4 + 6 * tau5;
      const ds = (30 * tau2 - 60 * tau3 + 30 * tau4) / this.duration;
      const d2s = (60 * tau - 180 * tau2 + 120 * tau3) / (this.duration * this.duration);

      const q = {};
      const v = {};
      const a = {};

      ['theta1', 'theta2', 'theta3'].forEach(joint => {
        const delta = this.qEnd[joint] - this.qStart[joint];
        q[joint] = this.qStart[joint] + delta * s;
        v[joint] = delta * ds;
        a[joint] = delta * d2s;
      });

      return { q, v, a, tau, done: tau >= 1.0 };
    }
  }

  // Export to global scope
  const Kinematics = {
    ROBOT_PARAMS,
    clamp,
    radToDeg,
    degToRad,
    getDHMatrix,
    multiply4x4,
    transformPoint,
    forwardKinematics,
    inverseKinematics,
    CubicTrajectory,
    QuinticTrajectory
  };

  if (typeof module !== 'undefined' && module.exports) {
    module.exports = Kinematics;
  } else {
    global.Kinematics = Kinematics;
  }

})(typeof window !== 'undefined' ? window : this);
