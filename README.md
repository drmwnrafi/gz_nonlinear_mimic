# Gazebo Non-Linear Mimic Plugin

A Gazebo Harmonic system plugin that drives a follower joint using a user-defined mathematical expression based on other mimic joints.

---

# Installation
```bash
git clone https://github.com/drmwnrafi/gz_nonlinear_mimic.git
cd gz_nonlinear_mimic
chmod +x install.sh
./install.sh
```
---

## SDF Parameters

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `joint` | `string` | Yes | Name of the follower joint to be controlled |
| `eq` | `string` | Yes | Mathematical expression for target position (exprtk syntax) |
| `n_mimic` | `int` | Yes | Number of mimic joints referenced in the expression |
| `mimicJoint1` … `mimicJointN` | `string` | Yes | Names of joints whose positions feed into the expression |
| `kp`, `ki`, `kd` | `double` | No | PID gains for position control (default: 50.0, 0.0, 5.0) |
| `max_torque` | `double` | No | Torque saturation limit (default: 100.0) |
| `debug` | `bool` | No | Enable periodic debug output (default: false) |

> Note: Mimic joints are exposed as `mimicJoint1`, `mimicJoint2`, … `mimicJointN` in the expression. 

Natively supported :
```text
Operators:  +  -  *  /  ^  %  ==  !=  <  <=  >  >=
Functions:  sin cos tan asin acos atan sqrt abs floor ceil pow min max
Constants:  pi e
```
---

# Example
```xml
<plugin filename="libNonLinearMimic.so" name="gz::sim::NonLinearMimic">
  <joint>wrist_joint</joint>
  <eq>2.0 * (mimicJoint1 - mimicJoint2)</eq>
  <n_mimic>2</n_mimic>
  <mimicJoint1>slider_right_joint</mimicJoint1>
  <mimicJoint2>slider_left_joint</mimicJoint2>
  <kp>100.0</kp>
  <kd>10.0</kd>
  <max_torque>50.0</max_torque>
</plugin>
```
