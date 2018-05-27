#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"
#include "Simulation/QuadDynamics.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

extern const V3F gravity;

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;

#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();

  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
  kdPosZ = config->Get(_config + ".kdPosZ", 0);
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to
  //   individual motor thrust commands
  // INPUTS:
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS:
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //! \todo DONE
  //printf("collThrustCmd: %f maxMotorThrust: %f \n", collThrustCmd, maxMotorThrust);

  float l = L / sqrt(2);
  float f_total = collThrustCmd ;  // [N]

  // Equations of the generated moments
  // moment = F * l

  float f_rot_x = momentCmd.x / l; // f0 + f3 - f1 -f2
  float f_rot_y = momentCmd.y / l; // f0 + f1 - f2 -f3
  float f_rot_z = momentCmd.z / kappa;

  //By solving these 4 linear equations
  // f_total =  f0 + f1 + f2 + f3
  // f_rot_x =  f0 - f1 + f2 - f3
  // f_rot_y =  f0 + f1 - f2 - f3
  // f_rot_z = -f0 + f1 + f2 - f3

  // we get these solutions
  float f0 = (f_total + f_rot_x + f_rot_y - f_rot_z) / 4.f ;
  float f2 = (f_total + f_rot_x - f_rot_y + f_rot_z) / 4.f ;
  float f1 = (f_total - f_rot_x + f_rot_y + f_rot_z) / 4.f ;
  float f3 = (f_total - f_rot_x - f_rot_y - f_rot_z) / 4.f ;

  cmd.desiredThrustsN[0] = CONSTRAIN( f0, minMotorThrust, maxMotorThrust); // front left
  cmd.desiredThrustsN[1] = CONSTRAIN( f1, minMotorThrust, maxMotorThrust); // front right
  cmd.desiredThrustsN[2] = CONSTRAIN( f2, minMotorThrust, maxMotorThrust); // rear left
  cmd.desiredThrustsN[3] = CONSTRAIN( f3, minMotorThrust, maxMotorThrust); // rear right

  assert(cmd.desiredThrustsN[0] <= maxMotorThrust);
  assert(cmd.desiredThrustsN[1] <= maxMotorThrust);
  assert(cmd.desiredThrustsN[2] <= maxMotorThrust);
  assert(cmd.desiredThrustsN[3] <= maxMotorThrust);
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}
/*! \brief Calculate body rate control

*/
V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS:
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS:
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //! \todo
  auto pqr_err = pqrCmd - pqr;

  momentCmd = V3F(Ixx, Iyy, Izz) * this->kpPQR * pqr_err;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS:
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS:
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //! \todo DONE

  // collective acceleration, the thrust is negative direction of z-axis
  auto coll_acc = -1 * collThrustCmd / mass; //[m/s^2] = N / kg
  // z(0) is the ground. For a hover we need a negative thrust

  // direction of the collective thrust in the inertial frame

  auto R13 = CONSTRAIN(accelCmd.x / coll_acc, -maxTiltAngle, maxTiltAngle);
  auto R23 = CONSTRAIN(accelCmd.y / coll_acc, -maxTiltAngle, maxTiltAngle);

  V3F R_Pcontrol;
  R_Pcontrol.x = this->kpBank * (R13 - R(0,2) );
  R_Pcontrol.y = this->kpBank * (R23 - R(1,2));
  R_Pcontrol.z = 0;

  Mat3x3F R_dot;
  R_dot(0,0) = R(1,0);
  R_dot(0,1) = -R(0,0);
  R_dot(1,0) = R(1,1);
  R_dot(1,1) = -R(0,1);
  R_dot = R_dot / R(2,2);

  // pc, qc as the angular velocities i nthe body frame
  pqrCmd = R_dot * R_Pcontrol;
  //printf("pqrCmd.p: %f, q: %f r: %f\n", pqrCmd.x, pqrCmd.y, pqrCmd.z);


  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical
  //   acceleration feed-forward command
  // INPUTS:
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS:
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust_cmd = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //! \todo DONE

  auto z_err = posZCmd - posZ;
  //printf("z_err: %f, posZCmd: %f, posZ: %f\n", z_err, posZCmd, posZ);
  // constrain the velocity command
  auto z_dot_err = velZCmd - velZ;
  //printf("z_dot_err: %f, velZCmd: %f, velZ %f\n", z_dot_err, velZCmd, velZ);
  auto pcontrol = kpPosZ * z_err;
  // Since velocity is the derivative of the pose, for D control we can use the same gain as P control
  auto dcontrol = kpPosZ * z_dot_err;

  // Note, the integral term ACCUMULATE the error over time. Thus we need +=.
  integratedAltitudeError += z_err * dt;
  auto icontrol = KiPosZ * integratedAltitudeError;

  auto acc_ff = accelZCmd;
  auto acc_cmd = pcontrol + icontrol + dcontrol + acc_ff;

  auto b_z = R(2,2);

  // Note, gravity is in the z positive direction. thus +g
  auto c_acc = (acc_cmd - gravity[2]) / b_z;
  // constrain the collective acceleration
  c_acc = CONSTRAIN(c_acc, -maxAscentRate / dt, maxAscentRate / dt);
  thrust_cmd = mass * -c_acc;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return thrust_cmd;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS:
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations.
  //     the Z component should be 0
  // HINTS:
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //! \todo DONE
  // Speed limit. Note, the input param velCmd will be modified
  velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
  velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);

  auto pos_err = posCmd - pos;
  auto vel_err = velCmd - vel;

  accelCmd += this->kpPosXY * pos_err+ this->kpVelXY * vel_err;

  accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
  accelCmd.y  = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
  accelCmd.z = 0;

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  //printf("pos_err.x: %f, y:%f vel_err.x: %f, y: %f\n", pos_err.x, pos_err.y, vel_err.x, vel_err.y);
  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS:
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS:
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b].
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //! \todo DONE
  // based on the python code solution https://github.com/udacity/FCND-Controls/blob/solution/controller.py
  auto restr_yawCmd = fmodf(yawCmd, 2.f * F_PI);
  auto yaw_err = restr_yawCmd - yaw;

  if (yaw_err > F_PI)
  {
    yaw_err = yaw_err - 2.f * F_PI;
  }
  else if (yaw_err < -F_PI)
  {
    yaw_err = yaw_err + 2.f * F_PI;
  }

  yawRateCmd = this->kpYaw * yaw_err;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);
  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);

  // check the constrain, e.g min: 2.160000, max: 16.240000. Hence, convert collThrustCmd to positive value in the CONSTRAIN function
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);

  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);

  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}


