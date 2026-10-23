#Python script to generate a human model with a exoskeleton coupled
#from a already scaled human model, using PiG markers for positioning

import opensim as osim
import numpy as np
import aid_functions as af

# Add to the class
osim.Rotation.to_numpy = af.r_to_numpy

"""
    FIRST PART:
    LOADING THE MODEL AND GETTING THE OBJECTS INTO PYTHON
"""
# Load the model we will add the exo and initiate it on standing still position
# The state is important for other position based functions. Specially during motion
# Since we are not doing that, we can just initate as is.
model = osim.Model("subject_walk_contact.osim")
model.finalizeConnections()
state = model.initSystem()

# Extracting marker set and its data. Making python dictionary for ease of use.
# The model expected is the Plug-in-gait marker set - See vicon for more details.
MarkerSet = {}
for Marker in model.getMarkerSet(): 
    # print("Marker Name:", Marker.getName()) 
    # print("Marker sockets:", Marker.getParentFrame().getName()) 
    # print(Marker.get_location()) 
    # print(Marker.getLocationInGround(state))
    # k = Marker.getLocationInGround(state)
    # narray = np.array([k.get(0), k.get(1), k.get(2)])
    # print(narray)
    MarkerSet[Marker.getName()] = Marker

# Creating the MarkerConvention class to handle different naming styles, see aid_functions.py for more details.
markers = af.MarkerConventions()
markers.detect_convention(model)
# Extracting the Frames of relevant joints. 
# Notice: This might not be available in every model
# but it is in the models I am currently using. Adapt as needed.

JointFrames = {
    "LJC" : model.getComponent("/jointset/back/pelvis_offset"),
    "HJC_R" : model.getComponent("/jointset/hip_r/femur_r_offset"),
    "HJC_L" : model.getComponent("/jointset/hip_l/femur_l_offset"),
    "KJC_R" : model.getComponent("/jointset/walker_knee_r/tibia_r_offset"),
    "KJC_L" : model.getComponent("/jointset/walker_knee_l/tibia_l_offset"),
    "AJC_R" : model.getComponent("/jointset/ankle_r/talus_r_offset"),
    "AJC_L" : model.getComponent("/jointset/ankle_l/talus_l_offset"),
}

print("Checando HJC_R")
print(JointFrames['HJC_R'].getName())
transform = JointFrames['HJC_R'].getTransformInGround(state)
af.print_transform(transform)

#Define list containing the naming convention of the markers. Defined in aid_functions.py
"""
    SECOND PART:
    USING THE DATA TO CALCULATE RELEVANT POSITIONS FOR EXO POSITIONING
"""
# Calculating relevant positions
# Naming convention: If a value, direct naming
# if dealing with a point in relation to ground we will have:
# Point_p -> position
# Point_r -> orientation/rotation
# Pelvis:
LJC_r = JointFrames['LJC'].getTransformInGround(state).R().to_numpy()
LJC_p = JointFrames['LJC'].getTransformInGround(state).p().to_numpy()
# Pelvis position in Y
Pelvis_Y = MarkerSet[markers.right_PSIS].getLocationInGround(state).get(1)
#  Pelvis width - While we can use rASIS - lASIS. I believe measuring from the thigh is better,
# we don't want just the bone measurement, but rather the body shape.

Pelvis_w = MarkerSet[markers.right_thigh4].getLocationInGround(state).get(2) - MarkerSet[markers.left_thigh3].getLocationInGround(state).get(2)
#  Pelvis length - Front to back
Pelvis_l = MarkerSet[markers.right_ASIS].getLocationInGround(state).get(0) - MarkerSet[markers.right_PSIS].getLocationInGround(state).get(0)

# print('Pelvis_Y', Pelvis_Y)
# print('Pelvis_w', Pelvis_w)
# print('Pelvis_l', Pelvis_l)

# Right leg:
#  Hip Joint Center
# Currently using from model. Implement calculation from markers later
R_HJC_r = JointFrames['HJC_R'].getTransformInGround(state).R().to_numpy()
R_HJC_p = JointFrames['HJC_R'].getTransformInGround(state).p().to_numpy()


#  Knee Joint Center
# Currently using from model. Implement calculation from markers later
R_KJC_r = JointFrames['KJC_R'].getTransformInGround(state).R().to_numpy()
R_KJC_p = JointFrames['KJC_R'].getTransformInGround(state).p().to_numpy()

#  Ankle Joint
# Currently using from model. Implement calculation from markers later
R_AJC_r = JointFrames['AJC_R'].getTransformInGround(state).R().to_numpy()
R_AJC_p = JointFrames['AJC_R'].getTransformInGround(state).p().to_numpy()

# # X and Y from the ankle and Calc markers, getting mean values. Z from toe marker
# x_sum = MarkerSet['r_ankle'].getLocationInGround(state).get(0)+MarkerSet['r_mankle'].getLocationInGround(state).get(0)
# y_sum = MarkerSet['r_ankle'].getLocationInGround(state).get(1)+MarkerSet['r_mankle'].getLocationInGround(state).get(1)+MarkerSet['r_calc'].getLocationInGround(state).get(1)
# R_AJC_p = np.array([x_sum/2, 
#                     y_sum/3,
#                     MarkerSet['r_toe'].getLocationInGround(state).get(2)])

# Left leg:
#  Hip Joint Center
# Currently using from model. Implement calculation from markers later
L_HJC_r = JointFrames['HJC_L'].getTransformInGround(state).R().to_numpy()
L_HJC_p = JointFrames['HJC_L'].getTransformInGround(state).p().to_numpy()

#  Knee Joint Center
# Currently using from model. Implement calculation from markers later
L_KJC_r = JointFrames['KJC_L'].getTransformInGround(state).R().to_numpy()
L_KJC_p = JointFrames['KJC_L'].getTransformInGround(state).p().to_numpy()

#  Ankle Joint
# Currently using from model. Implement calculation from markers later
L_AJC_r = JointFrames['AJC_L'].getTransformInGround(state).R().to_numpy()
L_AJC_p = JointFrames['AJC_L'].getTransformInGround(state).p().to_numpy()


#  Thigh offset
# Not exactly width, rather width starting from HJC, used to estimate Exo position. Mean of both legs.
thigh_os = (abs(R_HJC_p[2] - MarkerSet[markers.right_thigh4].getLocationInGround(state).get(2))
           +abs(L_HJC_p[2] - MarkerSet[markers.left_thigh3].getLocationInGround(state).get(2)))/2
# Thigh offset length
thigh_os_l = (abs(R_HJC_p[1] - MarkerSet[markers.right_knee].getLocationInGround(state).get(1))
           +abs(L_HJC_p[1] - MarkerSet[markers.left_knee].getLocationInGround(state).get(1)))/2

#  Knee offset
# Same ideia
knee_os = (abs(R_KJC_p[2]  - MarkerSet[markers.right_knee].getLocationInGround(state).get(2))
           +abs(L_KJC_p[2]  - MarkerSet[markers.left_knee].getLocationInGround(state).get(2)))/2
knee_os_l = (abs(R_KJC_p[1]  - MarkerSet[markers.right_ankle].getLocationInGround(state).get(1))
           +abs(L_KJC_p[1]  - MarkerSet[markers.left_ankle].getLocationInGround(state).get(1)))/2

#  Ankle offset
ankle_os = (abs(R_AJC_p[2]  - MarkerSet[markers.right_ankle].getLocationInGround(state).get(2))
           +abs(L_AJC_p[2]  - MarkerSet[markers.left_ankle].getLocationInGround(state).get(2)))/2

#  Feet position
feet_y = MarkerSet[markers.right_MT5].getLocationInGround(state).get(1)
feet_length = MarkerSet[markers.right_toe].getLocationInGround(state).get(0)-MarkerSet[markers.right_heel].getLocationInGround(state).get(0)
# The marker for feet_y is the MT5, however, in convention 1 it is not on the bottom of the
#foot. As such a small correction is needed. Be mindful of this for other conventions.
if markers.active_index == 0:
    feet_y = feet_y - 0.05
"""
    THIRD PART:
    DEFINING OUR EXO PARAMETERS AND BODIES
"""
# We will define our exo with 11 bodies as follows:
# 1 for back - welded to human
exo_back_Weight = 1
exo_back_CoM = osim.Vec3(0)
exo_back_Inertia = osim.Inertia(0.1, 0.1, 0.1)
# 1 for connector from back to waist
exo_back_2_waist_Weight = 1
exo_back_2_waist_CoM = osim.Vec3(0)
exo_back_2_waist_Inertia = osim.Inertia(0.1, 0.1, 0.1)
# 1 for waist (around hip), has lumbar rotations
exo_waist_Weight = 1
exo_waist_CoM = osim.Vec3(0)
exo_waist_Inertia = osim.Inertia(0.1, 0.1, 0.1)
# 2 for the connectors between waist and thigh
exo_waist_2_thigh_Weight = 1
exo_waist_2_thigh_CoM = osim.Vec3(0)
exo_waist_2_thigh_Inertia = osim.Inertia(0.1, 0.1, 0.1)
# 2 for the thighs
exo_thigh_Weight = 1
exo_thigh_CoM = osim.Vec3(0)
exo_thigh_Inertia = osim.Inertia(0.1, 0.1, 0.1)
# 2 for the shanks
exo_shank_Weight = 1
exo_shank_CoM = osim.Vec3(0)
exo_shank_Inertia = osim.Inertia(0.1, 0.1, 0.1)
# 2 for the feet
exo_feet_Weight = 1
exo_feet_CoM = osim.Vec3(0)
exo_feet_Inertia = osim.Inertia(0.1, 0.1, 0.1)

exo_back = osim.Body("exo_back", exo_back_Weight, exo_back_CoM, exo_back_Inertia)
# model.addBody(exo_back)
exo_back_2_waist =osim.Body("exo_back_2_waist", exo_back_2_waist_Weight, exo_back_2_waist_CoM, exo_back_2_waist_Inertia)
# model.addBody(exo_back_2_waist)
exo_waist = osim.Body("exo_waist", exo_waist_Weight, exo_waist_CoM, exo_waist_Inertia)
# model.addBody(exo_waist)
exo_waist_2_thigh_R =osim.Body("exo_waist_2_thigh_R", exo_waist_2_thigh_Weight, exo_waist_2_thigh_CoM, exo_waist_2_thigh_Inertia)
# model.addBody(exo_waist_2_thigh_R)
exo_waist_2_thigh_L =osim.Body("exo_waist_2_thigh_L", exo_waist_2_thigh_Weight, exo_waist_2_thigh_CoM, exo_waist_2_thigh_Inertia)
# model.addBody(exo_waist_2_thigh_L)
exo_R_thigh = osim.Body("exo_R_thigh", exo_thigh_Weight, exo_thigh_CoM, exo_thigh_Inertia)
# model.addBody(exo_R_thigh)
exo_L_thigh = osim.Body("exo_L_thigh", exo_thigh_Weight, exo_thigh_CoM, exo_thigh_Inertia)
# model.addBody(exo_L_thigh)
exo_R_shank = osim.Body("exo_R_shank", exo_shank_Weight, exo_shank_CoM, exo_shank_Inertia)
# model.addBody(exo_R_shank)
exo_L_shank = osim.Body("exo_L_shank", exo_shank_Weight, exo_shank_CoM, exo_shank_Inertia)
# model.addBody(exo_L_shank)
exo_R_feet = osim.Body("exo_R_feet", exo_feet_Weight, exo_feet_CoM, exo_feet_Inertia)
# model.addBody(exo_R_feet)
exo_L_feet = osim.Body("exo_L_feet", exo_feet_Weight, exo_feet_CoM, exo_feet_Inertia)
# model.addBody(exo_L_feet)

# Other relevant definitions:
powerpack_size = osim.Vec3(0.01, 0.05, 0.05)
"""
    FOURTH PART:
    CONSTRUCTING THE EXO
"""
# Welding the exo back to the torso - This a simplification. We should change to a custom joint with small range of motion
#to better represent the real coupling of the exo to the human. But for now, we will just weld it for simplicity.
# NOTA: ARRUMAR OFFSETS DE ACORDO COM MARCADORES.
parent = model.getBodySet().get("torso")
safety = 0.01
if markers.active_index == 0:
    exo_back_offset_x = -MarkerSet[markers.S2].getLocationInGround(state).get(0)+parent.getTransformInGround(state).p().get(0)
    exo_back_offset_y = -(parent.getTransformInGround(state).p().get(1)-MarkerSet[markers.S2].getLocationInGround(state).get(1))*3
elif markers.active_index == 1:
    exo_back_offset_x = -MarkerSet[markers.C7].getLocationInGround(state).get(0)+parent.getTransformInGround(state).p().get(0)
    exo_back_offset_y = -(parent.getTransformInGround(state).p().get(1)-MarkerSet[markers.C7].getLocationInGround(state).get(1))
joint = osim.WeldJoint(
    "exo_back_2_human",
    parent,                       # parent PhysicalFrame (e.g. model.getBodySet().get("torso"))
    osim.Vec3(0, 0, 0),           # locationInParent
    osim.Vec3(0, 0, 0),           # orientationInParent (XYZ body-fixed Euler angles)
    exo_back,                     # child PhysicalFrame (o seu exo_back já adicionado via model.addBody(exo_back))
    osim.Vec3(exo_back_offset_x+safety, exo_back_offset_y, 0),     # locationInChild
    osim.Vec3(0, 0, 0)            # orientationInChild
)

brick = osim.Brick(powerpack_size)
brick.setColor(osim.Vec3(1, 1, 0))
exo_back.attachGeometry(brick)
model.addBody(exo_back)
model.addJoint(joint)

# Connecting exo_back to its exo_back_2_waist connector. It uses two simple pinjoints. But first we need to define the distance from
#the powerpack to the exowaist
model.finalizeConnections()
state = model.initSystem()
exo_back_p = exo_back.getTransformInGround(state).p().to_numpy()
exo_back_R = exo_back.getTransformInGround(state).R().to_numpy()
transform = (LJC_p - exo_back_p) #This gets us the position of the LJC in relation to the parent body
#The position of the exowaist will be the halfway to the LJC. So the center of exob2w connector will be a quarter of this.

joint = osim.PinJoint(
    "exo_b2w_bjoint",
    exo_back,
    osim.Vec3(0,-powerpack_size[1],0), 
    osim.Vec3(0, 0, 0), 
    exo_back_2_waist,
    osim.Vec3(0, -transform[1]/4, 0), 
    osim.Vec3(0, 0, 0), 
)


model.addBody(exo_back_2_waist)
model.addJoint(joint)

#Set range
joint.getCoordinate(0).set_range(1, -1)
joint.getCoordinate(0).set_range(1, 3.5)
joint.getCoordinate(0).setDefaultClamped(True)

# Deprecated, but kept for refernece on custom joint creation.
# # Custom joint of exo_back_2_waist to the waist
# # This joint may look simple at first glance. But it must be defined as a 4DOF joint
# # It has 3 rotational axis, with the Y axis being offset from the body.
# # It also has a translation on the Y axis

# model.finalizeConnections()
# state = model.initSystem()
# exo_back_p = exo_back.getTransformInGround(state).p().to_numpy()
# exo_back_R = exo_back.getTransformInGround(state).R().to_numpy()
# transform = (LJC_p - exo_back_p) #This gets us the position of the LJC in relation to the parent body

# # To create a custom joint we first need to define its spatial transform.
# # Create the SpatialTransform
# st = osim.SpatialTransform()
# # By default, all 6 axis are locked, and are in order XYZ rotation then XYZ translation
# # We are going to use the order ZXY rotation then  Y translation. This is to be in line
# # with the human models of OpenSim.

# # --- Z-Rotation (Axis 0) FLEXION ---
# # Get the first rotational axis
# axis0 = st.getTransformAxis(0)
# axis0.set_coordinates(0, 'exo_back_flexion_R')
# axis0.setAxis(osim.Vec3(0, 0, 1))
# axis0.set_function(osim.LinearFunction(1,0))

# # --- X-Rotation (Axis 1) bending ---
# # Get the second rotational axis
# axis1 = st.getTransformAxis(1)
# axis1.set_coordinates(0, 'exo_back_bending_R')
# axis1.setAxis(osim.Vec3(1, 0, 0))
# axis1.set_function(osim.LinearFunction(1,0))

# # --- Y-Rotation (Axis 2) ROTATION ---
# axis2 = st.getTransformAxis(2)
# axis2.set_coordinates(0, 'exo_back_rot_R')
# axis2.setAxis(osim.Vec3(0, 1, 0))
# axis2.set_function(osim.LinearFunction(1,0))

# # --- Y-Translation (Axis 3) ---
# axis3 = st.getTransformAxis(3)
# axis3.set_coordinates(0, 'exo_back_trans_R')
# axis3.setAxis(osim.Vec3(0, 1, 0))
# axis3.set_function(osim.LinearFunction(1,0))

# # --- X-translation (Axis 4) ---
# # Normally, axis 4 is the Y axis. Since we used the axis 3 for the y
# # we need to move the X axis to the 4 axis.
# axis4 = st.getTransformAxis(4)
# axis4.setAxis(osim.Vec3(1, 0, 0))


# joint = osim.CustomJoint(
#     "exo_lumbar_joint",
#     exo_back,
#     osim.Vec3(transform[0], transform[1], transform[2]),  
#     osim.Vec3(0, 0, 0), 
#     exo_waist,
#     osim.Vec3(transform[0], 0, transform[2]),  
#     osim.Vec3(0, 0, 0),
#     st
# )

# Defining a pin joint connecting the exo_back_2_waist to the exo_waist.
model.finalizeConnections()
state = model.initSystem()
exo_back_p = exo_back.getTransformInGround(state).p().to_numpy()
exo_back_R = exo_back_2_waist.getTransformInGround(state).R().to_numpy()
transform = (LJC_p - exo_back_p) #This gets us the position of the LJC in relation to the parent body
offset = 0.01


joint = osim.PinJoint(
    "exo_b2w_wjoint",
    exo_back_2_waist,
    osim.Vec3(0, transform[1]/4,0),  
    osim.Vec3(0, 0, 0), 
    exo_waist,
    osim.Vec3(0, offset, 0),  
    osim.Vec3(0, 0, 0),
)
brick = osim.Brick(osim.Vec3(0.02, 0.01, Pelvis_w/2+safety))
brick.setColor(osim.Vec3(1, 1, 0))
exo_waist.attachGeometry(brick)
model.addBody(exo_waist)
model.addJoint(joint)

# #Set range for axis
# max_range = [2.5, 2.5, 2.5, 5]
# min_range = [-2.5, -2.5, -2.5, -2.5]
# for i in range(4):
#     joint.getCoordinate(i).set_range(0, min_range[i])
#     joint.getCoordinate(i).set_range(1, max_range[i])
#     joint.getCoordinate(i).setDefaultClamped(True)

#Set range
joint.getCoordinate(0).set_range(1, -1)
joint.getCoordinate(0).set_range(1, 3.5)
joint.getCoordinate(0).setDefaultClamped(True)

# Now that we have powerpack-b2w-exowaist we can define the geomtry of the b2w body:
model.finalizeConnections()
state = model.initSystem()
# First, get the distance from the frames of powerpack to frame of exo_waist
dist = exo_back.getTransformInGround(state).p().to_numpy() - exo_waist.getTransformInGround(state).p().to_numpy()
# now fix for their own length, so we get the distance between the two bodies, and not between the frames
dist[1] = dist[1] - powerpack_size[1]-offset
# Now the geomtry
brick = osim.Brick(osim.Vec3(0.01, dist[1]/2, 0.01))
brick.setColor(osim.Vec3(1, 1, 0))
exo_back_2_waist.attachGeometry(brick)

# Before going to the hip: The hip joint will be conected to a different body welded on the waist, and not its central one.
# think of it as having two offsets. This is important to reduce how dependent the joint is to the waist
# relative position. Allowing more independence between each side. These are massless.

# Find the transform of the new body, expressed in the exo-waist frame:
# We will position it relative to the hip on the X axis. And take the
# person width for the Z axis. Y will be on the same level as the exo-waist
# Position of parent body to ground
model.finalizeConnections()
state = model.initSystem()
exo_waist_p = exo_waist.getTransformInGround(state).p().to_numpy()

setup_w2h_p_R = osim.Vec3(Pelvis_l/2, 0, Pelvis_w/2) #Position of the center of the new body
a = osim.Transform(osim.Rotation() ,setup_w2h_p_R)
# Create a new Body on the exo-waist. Lets visualize it aswell
exo_w2h_R = osim.Body("exo_w2h_R", 0, osim.Vec3(0,0,0), osim.Inertia(0,0,0))
brick = osim.Brick(osim.Vec3(Pelvis_l/2, 0.01, 0.01))
brick.setColor(osim.Vec3(0, 1, 1))
exo_w2h_R.attachGeometry(brick)

joint = osim.WeldJoint(
    "exo_waist2_w2h_R",
    exo_waist,                       # parent PhysicalFrame (e.g. model.getBodySet().get("torso"))
    setup_w2h_p_R,           # locationInParent
    osim.Vec3(0, 0, 0),           # orientationInParent (XYZ body-fixed Euler angles)
    exo_w2h_R,                     # child PhysicalFrame (o seu exo_back já adicionado via model.addBody(exo_back))
    osim.Vec3(0, 0, 0),     # locationInChild
    osim.Vec3(0, 0, 0)            # orientationInChild
)
model.addBody(exo_w2h_R)
model.addJoint(joint)

#Now repeat for the left side
setup_w2h_p_L = osim.Vec3(Pelvis_l/2, 0, -Pelvis_w/2) #Position of the center of the new body
a = osim.Transform(osim.Rotation() ,setup_w2h_p_R)
exo_w2h_L = osim.Body("exo_w2h_L", 0, osim.Vec3(0,0,0), osim.Inertia(0,0,0))
brick = osim.Brick(osim.Vec3(Pelvis_l/2, 0.01, 0.01))
brick.setColor(osim.Vec3(1, 0, 1))
exo_w2h_L.attachGeometry(brick)

joint = osim.WeldJoint(
    "exo_waist2_w2h_L",
    exo_waist,                       # parent PhysicalFrame (e.g. model.getBodySet().get("torso"))
    setup_w2h_p_L,           # locationInParent
    osim.Vec3(0, 0, 0),           # orientationInParent (XYZ body-fixed Euler angles)
    exo_w2h_L,                     # child PhysicalFrame (o seu exo_back já adicionado via model.addBody(exo_back))
    osim.Vec3(0, 0, 0),     # locationInChild
    osim.Vec3(0, 0, 0)            # orientationInChild
)
model.addBody(exo_w2h_L)
model.addJoint(joint)

# RIGHT LEG

# Hip
# Divided in two parts -  One for adduction and rotation, a second for flexing
# Hip Adduction+Rot
model.finalizeConnections()
state = model.initSystem()

# Position of parent frame
parent = exo_w2h_R
parent_p = parent.getTransformInGround(state).p().to_numpy()

# Joint Position in relation to parent body (align x to HJC, others are 0)
transform_x = R_HJC_p[0] - parent_p[0]

# Joint Position in relation to child body (How long is the link)
transform_y = R_HJC_p[1] - parent_p[1]

# We are going to make a custom joint of 2 DoF. X and Y rotation.
# Create the SpatialTransform
hipAR_st = osim.SpatialTransform()

# --- X-Rotation (Axis 0) ADDUCTION ---
# Get the second rotational axis
axis0 = hipAR_st.getTransformAxis(0)
axis0.set_coordinates(0, 'exo_hip_adduction_R')
axis0.setAxis(osim.Vec3(1, 0, 0))
axis0.set_function(osim.LinearFunction(1,0))

# --- Y-Rotation (Axis 1) ROTATION ---
axis1 = hipAR_st.getTransformAxis(1)
axis1.set_coordinates(0, 'exo_hip_rot_R')
axis1.setAxis(osim.Vec3(0, 1, 0))
axis1.set_function(osim.LinearFunction(1,0))

#Since we used the axis in order, no further changes are needed.
# We can now create the joint.

joint = osim.CustomJoint(
    "exo_hip_AR_joint_R",
    parent,
    osim.Vec3(transform_x, 0, 0), 
    osim.Vec3(0, 0, 0), 
    exo_waist_2_thigh_R,
    osim.Vec3(0, -transform_y/2, 0), 
    osim.Vec3(0, 0, 0), 
    hipAR_st
)

brick = osim.Cylinder(0.01, transform_y/2)
brick.setColor(osim.Vec3(0, 1, 1))
exo_waist_2_thigh_R.attachGeometry(brick)
model.addBody(exo_waist_2_thigh_R)
model.addJoint(joint)

#Set range for axis
max_range = [1.52, 1.69]
min_range = [-1.8, -1.69]
for i in range(2):
    joint.getCoordinate(i).set_range(0, min_range[i])
    joint.getCoordinate(i).set_range(1, max_range[i])
    joint.getCoordinate(i).setDefaultClamped(True)
# Hip Flex
# Transform vector: R_HJC_r - setup_waist_2_hip_frame_R

model.finalizeConnections()
state = model.initSystem()

#Parent frame position
parent = exo_waist_2_thigh_R
parent_p = parent.getTransformInGround(state).p().to_numpy()

#Joint position in relation to parent body
transform = (R_HJC_p - parent_p)
transform[2] = transform[2]+thigh_os

joint = osim.PinJoint(
    "exo_hipF_joint_R",
    parent,
    osim.Vec3.createFromMat(transform), 
    osim.Vec3(0, 0, 0), 
    exo_R_thigh,
    osim.Vec3(0, thigh_os_l/2, 0), 
    osim.Vec3(0, 0, 0)
)

brick = osim.Cylinder(0.01, thigh_os_l/2+safety)
brick.setColor(osim.Vec3(0, 0.8, 0.8)) #Slightly darker for viewing
exo_R_thigh.attachGeometry(brick)
model.addBody(exo_R_thigh)
model.addJoint(joint)

#Set range
joint.getCoordinate(0).set_range(0, -1.5)
joint.getCoordinate(0).set_range(1, 3.1)
joint.getCoordinate(0).setDefaultClamped(True)

# Knee Joint

model.finalizeConnections()
state = model.initSystem()
exo_R_thigh_p = exo_R_thigh.getTransformInGround(state).p().to_numpy()

transform = (R_KJC_p - exo_R_thigh_p)
transform[2] = transform[2]+thigh_os

joint = osim.PinJoint(
    "exo_knee_joint_R",
    exo_R_thigh,
    osim.Vec3.createFromMat(transform), 
    osim.Vec3(0, osim.SimTK_PI, 0), 
    exo_R_shank,
    osim.Vec3(0, knee_os_l/2, 0), 
    osim.Vec3(0, osim.SimTK_PI, 0), 
)

brick = osim.Cylinder(0.01, knee_os_l/2+safety)
brick.setColor(osim.Vec3(0, 1, 1))
exo_R_shank.attachGeometry(brick)
model.addBody(exo_R_shank)
model.addJoint(joint)

#Set range
joint.getCoordinate(0).set_range(0, -1)
joint.getCoordinate(0).set_range(1, 3.5)
joint.getCoordinate(0).setDefaultClamped(True)

# Ankle Joint

model.finalizeConnections()
state = model.initSystem()
exo_R_shank_p = exo_R_shank.getTransformInGround(state).p().to_numpy()

transform = (R_AJC_p - exo_R_shank_p)
transform[2] = transform[2]

# Create the SpatialTransform
spatial_transform = osim.SpatialTransform()

# --- Z-Rotation (Axis 0) ---
# Get the first rotational axis
axis0 = spatial_transform.getTransformAxis(0)
# Set its DoF to be the 'rotZ' coordinate
axis0.set_coordinates(0, 'q_rz')
axis0.setAxis(osim.Vec3(0, 0, 1))
axis0.set_function(osim.LinearFunction(1,0))

# --- Y-Rotation (Axis 1) ---
# Get the second rotational axis
axis1 = spatial_transform.getTransformAxis(1)
# Set its DoF to be the 'rotY' coordinate
axis1.set_coordinates(0, 'q_ry')
axis1.setAxis(osim.Vec3(0, 1, 0))
axis1.set_function(osim.LinearFunction(1,0))

# --- X-Rotation (Axis 2) ---
# Correctly constructing ST -> Reordering the axis to avoid colinearity between axis
axis2 = spatial_transform.getTransformAxis(2)
axis2.setAxis(osim.Vec3(1, 0, 0))

# 3 (TransX), 4 (TransY), and 5 (TransZ)
# remain as default (constant, 0-value), which locks those DoFs.

joint = osim.CustomJoint("exo_ankle_joint_R",
                                exo_R_shank,
                                osim.Vec3.createFromMat(transform),
                                osim.Vec3(0, 0, -osim.SimTK_PI/3),
                                exo_R_feet,
                                osim.Vec3(0, R_AJC_p[1]-feet_y, 0),
                                osim.Vec3(0, 0, -osim.SimTK_PI/3),
                                spatial_transform)

brick = osim.Brick(osim.Vec3(0.1, 0.001, 0.1))
brick.setColor(osim.Vec3(0, 1, 1))
exo_R_feet.attachGeometry(brick)
model.addBody(exo_R_feet)
model.addJoint(joint)

#Set range for axis
max_range = [1.8, 1.69]
min_range = [-1.8, -1.69]
for i in range(2):
    joint.getCoordinate(i).set_range(0, min_range[i])
    joint.getCoordinate(i).set_range(1, max_range[i])
    joint.getCoordinate(i).setDefaultClamped(True)

# LEFT LEG
# Hip
# Divided in two parts -  One for adduction and rotation, a second for flexing
# Hip Adduction+Rot
model.finalizeConnections()
state = model.initSystem()

# Position of parent frame
parent = exo_w2h_L
parent_p = parent.getTransformInGround(state).p().to_numpy()

# Joint Position in relation to parent body (align x to HJC, others are 0)
transform_x = L_HJC_p[0] - parent_p[0]

# Joint Position in relation to child body (How long is the link)
transform_y = L_HJC_p[1] - parent_p[1]

# We are going to make a custom joint of 2 DoF. X and Y rotation.
# We have already created this spatial transform, lets just rename coordinates
axis0 = hipAR_st.getTransformAxis(0)
axis0.set_coordinates(0, 'exo_hip_adduction_L')
axis1 = hipAR_st.getTransformAxis(1)
axis1.set_coordinates(0, 'exo_hip_rot_L')

# We can now create the joint.

joint = osim.CustomJoint(
    "exo_hip_AR_joint_L",
    parent,
    osim.Vec3(transform_x, 0, 0), 
    osim.Vec3(0, osim.SimTK_PI, 0), 
    exo_waist_2_thigh_L,
    osim.Vec3(0, -transform_y/2, 0), 
    osim.Vec3(0, osim.SimTK_PI, 0), 
    hipAR_st
)

brick = osim.Cylinder(0.01, transform_y/2)
brick.setColor(osim.Vec3(1, 0, 1)) #Change color
exo_waist_2_thigh_L.attachGeometry(brick)
model.addBody(exo_waist_2_thigh_L)
model.addJoint(joint)

#Set range for axis
max_range = [1.52, 1.69]
min_range = [-1.8, -1.69]
for i in range(2):
    joint.getCoordinate(i).set_range(0, min_range[i])
    joint.getCoordinate(i).set_range(1, max_range[i])
    joint.getCoordinate(i).setDefaultClamped(True)

# Hip Flex
# Transform vector: R_HJC_r - setup_waist_2_hip_frame_R

model.finalizeConnections()
state = model.initSystem()

#Parent frame position
parent = exo_waist_2_thigh_L
parent_p = parent.getTransformInGround(state).p().to_numpy()

#Joint position in relation to parent body
transform = (L_HJC_p - parent_p)
transform[2] = transform[2]-thigh_os

joint = osim.PinJoint(
    "exo_hipF_joint_L",
    parent,
    osim.Vec3.createFromMat(transform), 
    osim.Vec3(0, 0, 0), 
    exo_L_thigh,
    osim.Vec3(0, thigh_os_l/2, 0), 
    osim.Vec3(0, 0, 0)
)

brick = osim.Cylinder(0.01, thigh_os_l/2+safety)
brick.setColor(osim.Vec3(0.8, 0, 0.8)) #Slightly darker for viewing
exo_L_thigh.attachGeometry(brick)
model.addBody(exo_L_thigh)
model.addJoint(joint)

#Set range
joint.getCoordinate(0).set_range(0, -1.5)
joint.getCoordinate(0).set_range(1, 3.1)

# Knee Joint

model.finalizeConnections()
state = model.initSystem()
exo_L_thigh_p = exo_L_thigh.getTransformInGround(state).p().to_numpy()

transform = (L_KJC_p - exo_L_thigh_p)
transform[2] = transform[2]-thigh_os

joint = osim.PinJoint(
    "exo_knee_joint_L",
    exo_L_thigh,
    osim.Vec3.createFromMat(transform), 
    osim.Vec3(0, osim.SimTK_PI, 0), 
    exo_L_shank,
    osim.Vec3(0, knee_os_l/2, 0), 
    osim.Vec3(0, osim.SimTK_PI, 0), 
)

brick = osim.Cylinder(0.01, knee_os_l/2+safety)
brick.setColor(osim.Vec3(1, 0, 1))
exo_L_shank.attachGeometry(brick)
model.addBody(exo_L_shank)
model.addJoint(joint)

#Set range
joint.getCoordinate(0).set_range(0, -1)
joint.getCoordinate(0).set_range(1, 3.5)
joint.getCoordinate(0).setDefaultClamped(True)

# Ankle Joint

model.finalizeConnections()
state = model.initSystem()
exo_L_shank_p = exo_L_shank.getTransformInGround(state).p().to_numpy()

transform = (L_AJC_p - exo_L_shank_p)
transform[2] = transform[2]

# Create the SpatialTransform
spatial_transform = osim.SpatialTransform()

# --- Z-Rotation (Axis 0) ---
# Get the first rotational axis
axis0 = spatial_transform.getTransformAxis(0)
# Set its DoF to be the 'rotZ' coordinate
axis0.set_coordinates(0, 'q_rz')
axis0.setAxis(osim.Vec3(0, 0, 1))
axis0.set_function(osim.LinearFunction(1,0))

# --- Y-Rotation (Axis 1) ---
# Get the second rotational axis
axis1 = spatial_transform.getTransformAxis(1)
# Set its DoF to be the 'rotY' coordinate
axis1.set_coordinates(0, 'q_ry')
axis1.setAxis(osim.Vec3(0, 1, 0))
axis1.set_function(osim.LinearFunction(1,0))

# --- X-Rotation (Axis 2) ---
# Correctly constructing ST -> Reordering the axis
axis2 = spatial_transform.getTransformAxis(2)
axis2.setAxis(osim.Vec3(1, 0, 0))

# 3 (TransX), 4 (TransY), and 5 (TransZ)
# remain as default (constant, 0-value), which locks those DoFs.

joint = osim.CustomJoint("exo_ankle_joint_L",
                                exo_L_shank,
                                osim.Vec3.createFromMat(transform),
                                osim.Vec3(0, 0, -osim.SimTK_PI/3),
                                exo_L_feet,
                                osim.Vec3(0, L_AJC_p[1]-feet_y, 0),
                                osim.Vec3(0, 0, -osim.SimTK_PI/3),
                                spatial_transform)

brick = osim.Brick(osim.Vec3(0.1, 0.001, 0.1))
brick.setColor(osim.Vec3(1, 0, 1))
exo_L_feet.attachGeometry(brick)
model.addBody(exo_L_feet)
model.addJoint(joint)

#Set range for axis
max_range = [1.8, 1.69]
min_range = [-1.8, -1.69]
for i in range(2):
    joint.getCoordinate(i).set_range(0, min_range[i])
    joint.getCoordinate(i).set_range(1, max_range[i])
    joint.getCoordinate(i).setDefaultClamped(True)

#With this we have finalized creating the exo bodies around the person. We will initialize the system now.

model.finalizeConnections()
state = model.initSystem()


"""
    FIFTH PART:
    EXO CONSTRAINS
"""

# The constraint will be a weld constraint on the feet, more specifically, on the heel.
# NOTA: TALVEZ SUBSTITUIR POR BUSHING FORCES -> SOFT CONSTRAINT
# https://simtk.org/api_docs/opensim/api_docs/classOpenSim_1_1BushingForce.html#details

# Get the two frames we want to connect
# 1. The person's existing heel frame
calcn_frame_r = model.getComponent("/jointset/subtalar_r/calcn_r_offset")

# Find the transform of the heel frame, expressed in the exo-foot frame
transform_in_exo = calcn_frame_r.findTransformBetween(state, exo_R_feet)

# Create a new PhysicalOffsetFrame on the exo-foot that is
# coincident with the heel frame
weld_frame_r = osim.PhysicalOffsetFrame("calcn_weld_frame_r",
                                      exo_R_feet,
                                      transform_in_exo)

# Add the new frame to the exo-foot
exo_R_feet.addComponent(weld_frame_r)

# Create the weld constraint
path_frame = exo_R_feet.getComponent('calcn_weld_frame_r').getAbsolutePathString()
weld_constraint_r = osim.WeldConstraint("weld_constraint_r",
                                        "/jointset/subtalar_r/calcn_r_offset",
                                        path_frame
                                        )

# Add the constraint to the model
model.addConstraint(weld_constraint_r)
model.finalizeConnections()
state = model.initSystem()
#Now repeat for the left
calcn_frame_l = model.getComponent("/jointset/subtalar_l/calcn_l_offset")
transform_in_exo = calcn_frame_l.findTransformBetween(state, exo_L_feet)
weld_frame_l = osim.PhysicalOffsetFrame("calcn_weld_frame_l",
                                      exo_L_feet,
                                      transform_in_exo)

exo_L_feet.addComponent(weld_frame_l)
path_frame = exo_L_feet.getComponent('calcn_weld_frame_l').getAbsolutePathString()
weld_constraint_l = osim.WeldConstraint("weld_constraint_l",
                                        "/jointset/subtalar_l/calcn_l_offset",
                                        path_frame
                                        )


model.addConstraint(weld_constraint_l)

"""
    SIXTH PART:
    EXO COUPLING FORCES
"""
# We will have a (linear or non-linear) spring damp coupling the thighs and shanks of the exo.
# F_spring = k1*x+k2*x^2+k3*x^3; Linear version: F_spring = k*x
# F_damp   = b*dx
# Linear version parameter values from: https://doi.org/10.1007/s11071-022-07876-8
k_thigh = 3.283*10**3; k_shank = 5.645*10**3
b_thigh = 0.367*10**3; b_shank = 0.418*10**3
# Non-linear version parameters: https://doi.org/10.1109/TOH.2024.3375295 
#FAZER ISSO

# Restin length
rlt = 0.1; rls = 0.1

# Define force expression:
# Available variables: `d` (distance − rest_length), `ddot` (rate)
# expr = f"{k1}*x + {k2}*x^2 + {k2}*x^3 + {b}*dx"
expr_t = f"{k_thigh}*({rlt}-d) + {b_thigh}*ddot"
expr_s = f"{k_shank}*({rls}-d) + {b_shank}*ddot"

# RIGHT THIGH
# Get bodies that are going to be used (model component and its path)
body1 = model.getBodySet().get('femur_r')
body1_path = model.getBodySet().get('femur_r').getAbsolutePathString()
body2 = exo_R_thigh
body2_path = exo_R_thigh.getAbsolutePathString()

# Position of the coupling system on each body (femur and exo_thigh).
# NOTE: FEMUR POSITIONING STARTS ON HIP JOINT CENTER. EXO_THIGH ON GEOMETRIC CENTER.
body1_p = osim.Vec3(0, -0.2, 0)
body2_p = osim.Vec3(0, 0, 0)

# Create force
coupling_thigh_r = osim.ExpressionBasedPointToPointForce(body1_path,
                                                         body1_p,
                                                         body2_path,
                                                         body2_p,
                                                         expr_t)
coupling_thigh_r.setName('coupling_thigh_r')
# Generate a path for visualizing the connection
force_path = osim.GeometryPath()
force_path.appendNewPathPoint('human_point',
                              body1,
                              body1_p)
force_path.appendNewPathPoint('exo_point',
                              body2,
                              body2_p)
force_path.setDefaultColor(osim.Vec3(0, 1, 0))
coupling_thigh_r.addComponent(force_path)

model.addForce(coupling_thigh_r)

# LEFT THIGH
# Get bodies that are going to be used (model component and its path)
body1 = model.getBodySet().get('femur_l')
body1_path = model.getBodySet().get('femur_l').getAbsolutePathString()
body2 = exo_L_thigh
body2_path = exo_L_thigh.getAbsolutePathString()

# Position of the coupling system on each body (femur and exo_thigh).
# NOTE: FEMUR POSITIONING STARTS ON HIP JOINT CENTER. EXO_THIGH ON GEOMETRIC CENTER.
body1_p = osim.Vec3(0, -0.2, 0)
body2_p = osim.Vec3(0, 0, 0)

# Create force
coupling_thigh_l = osim.ExpressionBasedPointToPointForce(body1_path,
                                                         body1_p,
                                                         body2_path,
                                                         body2_p,
                                                         expr_t)
coupling_thigh_l.setName('coupling_thigh_l')
# Generate a path for visualizing the connection
force_path = osim.GeometryPath()
force_path.appendNewPathPoint('human_point',
                              body1,
                              body1_p)
force_path.appendNewPathPoint('exo_point',
                              body2,
                              body2_p)
force_path.setDefaultColor(osim.Vec3(0, 1, 0))
coupling_thigh_l.addComponent(force_path)

model.addForce(coupling_thigh_l)

# RIGHT SHANK
# Get bodies that are going to be used (model component and its path)
body1 = model.getBodySet().get('tibia_r')
body1_path = model.getBodySet().get('tibia_r').getAbsolutePathString()
body2 = exo_R_shank
body2_path = exo_R_shank.getAbsolutePathString()

# Position of the coupling system on each body (femur and exo_thigh).
# NOTE: SHANK POSITIONING STARTS ON KNEE JOINT CENTER. EXO_THIGH ON GEOMETRIC CENTER.
body1_p = osim.Vec3(0, -0.2, 0)
body2_p = osim.Vec3(0, 0, 0)

# Create force
coupling_shank_r = osim.ExpressionBasedPointToPointForce(body1_path,
                                                         body1_p,
                                                         body2_path,
                                                         body2_p,
                                                         expr_t)
coupling_shank_r.setName('coupling_shank_r')
# Generate a path for visualizing the connection
force_path = osim.GeometryPath()
force_path.appendNewPathPoint('human_point',
                              body1,
                              body1_p)
force_path.appendNewPathPoint('exo_point',
                              body2,
                              body2_p)
force_path.setDefaultColor(osim.Vec3(0, 1, 0))
coupling_shank_r.addComponent(force_path)

model.addForce(coupling_shank_r)

# LEFT SHANK
# Get bodies that are going to be used (model component and its path)
body1 = model.getBodySet().get('tibia_l')
body1_path = model.getBodySet().get('tibia_l').getAbsolutePathString()
body2 = exo_L_shank
body2_path = exo_L_shank.getAbsolutePathString()

# Position of the coupling system on each body (femur and exo_thigh).
# NOTE: SHANK POSITIONING STARTS ON KNEE JOINT CENTER. EXO_THIGH ON GEOMETRIC CENTER.
body1_p = osim.Vec3(0, -0.2, 0)
body2_p = osim.Vec3(0, 0, 0)

# Create force
coupling_shank_l = osim.ExpressionBasedPointToPointForce(body1_path,
                                                         body1_p,
                                                         body2_path,
                                                         body2_p,
                                                         expr_t)
coupling_shank_l.setName('coupling_shank_l')
# Generate a path for visualizing the connection
force_path = osim.GeometryPath()
force_path.appendNewPathPoint('human_point',
                              body1,
                              body1_p)
force_path.appendNewPathPoint('exo_point',
                              body2,
                              body2_p)
force_path.setDefaultColor(osim.Vec3(0, 1, 0))
coupling_shank_l.addComponent(force_path)

model.addForce(coupling_shank_l)


model.finalizeConnections()
state = model.initSystem()
model.printToXML("test-output.osim")
print('yay')
