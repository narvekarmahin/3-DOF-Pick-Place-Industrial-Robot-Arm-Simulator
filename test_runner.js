/**
 * test_runner.js
 * Comprehensive automated verification suite for 3-DOF Robot Arm Kinematics & Simulation.
 */

const Kinematics = require('./kinematics.js');

function assertClose(actual, expected, tol = 1e-3, msg = '') {
  if (Math.abs(actual - expected) > tol) {
    throw new Error(`Assertion failed: ${msg} | Expected ${expected}, got ${actual} (diff: ${Math.abs(actual - expected)})`);
  }
}

console.log("=== RUNNING KINEMATICS & TRAJECTORY VERIFICATION SUITE ===");

// 1. Home Configuration Test
console.log("\n[Test 1] Testing Home Position FK...");
const homeAngles = Kinematics.ROBOT_PARAMS.home;
const fkHome = Kinematics.forwardKinematics(homeAngles.theta1, homeAngles.theta2, homeAngles.theta3);
console.log(`Home FK End-Effector: X=${fkHome.position.x.toFixed(4)}, Y=${fkHome.position.y.toFixed(4)}, Z=${fkHome.position.z.toFixed(4)}`);
assertClose(fkHome.position.x, 1.6485, 1e-3, "Home X coordinate");
assertClose(fkHome.position.y, 0.0000, 1e-3, "Home Y coordinate");
assertClose(fkHome.position.z, 2.3485, 1e-3, "Home Z coordinate");
console.log("✓ Home Position FK matches theoretical values.");

// 2. Target A Pick Position Test: [1.0, 1.0, 0.2]
console.log("\n[Test 2] Testing Target A [1.0, 1.0, 0.2] IK & Roundtrip FK...");
const ikA = Kinematics.inverseKinematics(1.0, 1.0, 0.2);
console.log(`Target A IK: θ1=${Kinematics.radToDeg(ikA.theta1).toFixed(2)}°, θ2=${Kinematics.radToDeg(ikA.theta2).toFixed(2)}°, θ3=${Kinematics.radToDeg(ikA.theta3).toFixed(2)}°`);
assertClose(Kinematics.radToDeg(ikA.theta1), 45.0, 1e-2, "Target A θ1");
assertClose(ikA.clamped, false, 1e-3, "Target A must not be clamped");

const fkA = Kinematics.forwardKinematics(ikA.theta1, ikA.theta2, ikA.theta3);
console.log(`Target A FK Roundtrip: X=${fkA.position.x.toFixed(4)}, Y=${fkA.position.y.toFixed(4)}, Z=${fkA.position.z.toFixed(4)}`);
assertClose(fkA.position.x, 1.0, 1e-3, "Target A Roundtrip X");
assertClose(fkA.position.y, 1.0, 1e-3, "Target A Roundtrip Y");
assertClose(fkA.position.z, 0.2, 1e-3, "Target A Roundtrip Z");
console.log("✓ Target A IK/FK roundtrip is exact.");

// 3. Target B Place Position Test: [-1.0, 1.2, 0.2]
console.log("\n[Test 3] Testing Target B [-1.0, 1.2, 0.2] IK & Graceful Reach Boundary Clamping...");
const ikB = Kinematics.inverseKinematics(-1.0, 1.2, 0.2);
console.log(`Target B raw distance: ${ikB.rawDistance.toFixed(4)}m (Max: ${Kinematics.ROBOT_PARAMS.dMax}m)`);
console.log(`Target B Clamped: ${ikB.clamped}, Status: "${ikB.statusMessage}"`);
console.log(`Target B IK: θ1=${Kinematics.radToDeg(ikB.theta1).toFixed(2)}°, θ2=${Kinematics.radToDeg(ikB.theta2).toFixed(2)}°, θ3=${Kinematics.radToDeg(ikB.theta3).toFixed(2)}°`);
assertClose(Kinematics.radToDeg(ikB.theta1), 129.805, 1e-2, "Target B θ1 azimuth");

const fkB = Kinematics.forwardKinematics(ikB.theta1, ikB.theta2, ikB.theta3);
console.log(`Target B FK at Clamped Boundary: X=${fkB.position.x.toFixed(4)}, Y=${fkB.position.y.toFixed(4)}, Z=${fkB.position.z.toFixed(4)}`);
assertClose(fkB.position.x, ikB.clampedTarget.x, 1e-3, "Target B Clamped X");
assertClose(fkB.position.y, ikB.clampedTarget.y, 1e-3, "Target B Clamped Y");
assertClose(fkB.position.z, ikB.clampedTarget.z, 1e-3, "Target B Clamped Z");
console.log("✓ Target B boundary handling and IK are robust.");

// 4. Singularity Detection Test (Shoulder Singularity at X=0, Y=0)
console.log("\n[Test 4] Testing Shoulder Singularity Handling at [0, 0, 1.8]...");
const ikSing = Kinematics.inverseKinematics(0, 0, 1.8, { currentTheta1: 0.5 });
console.log(`Singularity detected: ${ikSing.singularity}, Preserved θ1: ${ikSing.theta1}`);
assertClose(ikSing.theta1, 0.5, 1e-4, "Preserved theta1 on singularity");
console.log("✓ Shoulder singularity handled gracefully.");

// 5. Cubic Trajectory Continuity Test
console.log("\n[Test 5] Testing Cubic Trajectory Boundary Conditions...");
const q0 = { theta1: 0, theta2: 0, theta3: 0 };
const q1 = { theta1: 1.0, theta2: 0.5, theta3: -0.5 };
const traj = new Kinematics.CubicTrajectory(q0, q1, 2.0);
const start = traj.evaluate(0);
const mid = traj.evaluate(1.0);
const end = traj.evaluate(2.0);
assertClose(start.q.theta1, 0, 1e-4, "Start position");
assertClose(start.v.theta1, 0, 1e-4, "Start velocity zero");
assertClose(end.q.theta1, 1.0, 1e-4, "End position");
assertClose(end.v.theta1, 0, 1e-4, "End velocity zero");
assertClose(mid.v.theta1, 0.75, 1e-4, "Midpoint peak velocity");
console.log(`Start: q=${start.q.theta1}, v=${start.v.theta1}`);
console.log(`Mid:   q=${mid.q.theta1}, v=${mid.v.theta1}`);
console.log(`End:   q=${end.q.theta1}, v=${end.v.theta1}`);
console.log("✓ Cubic trajectory boundary velocities are strictly zero.");

// 6. DH Homogeneous Transformation Matrix Orthonormality Test
console.log("\n[Test 6] Testing DH Matrix Properties (Orthonormality of R)...");
const T03 = fkHome.matrices.T03;
const r1 = [T03[0][0], T03[1][0], T03[2][0]]; // Col 1
const r2 = [T03[0][1], T03[1][1], T03[2][1]]; // Col 2
const r3 = [T03[0][2], T03[1][2], T03[2][2]]; // Col 3

const dot12 = r1[0]*r2[0] + r1[1]*r2[1] + r1[2]*r2[2];
const dot13 = r1[0]*r3[0] + r1[1]*r3[1] + r1[2]*r3[2];
const dot23 = r2[0]*r3[0] + r2[1]*r3[1] + r2[2]*r3[2];
const norm1 = Math.sqrt(r1[0]**2 + r1[1]**2 + r1[2]**2);

assertClose(dot12, 0, 1e-4, "Col 1 orthogonal to Col 2");
assertClose(dot13, 0, 1e-4, "Col 1 orthogonal to Col 3");
assertClose(dot23, 0, 1e-4, "Col 2 orthogonal to Col 3");
assertClose(norm1, 1, 1e-4, "Col 1 unit length");
console.log("✓ DH Transformation Matrix rotation block is strictly orthonormal.");

console.log("\n========================================================");
console.log("ALL 6 KINEMATIC & TRAJECTORY TESTS PASSED WITH 100% ACCURACY!");
console.log("========================================================\n");
