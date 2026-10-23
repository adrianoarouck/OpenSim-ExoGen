#Python script to generate the virtual marker for the exo back and feet
#in order to better estimate exo position in the simulation

import opensim as osim
import numpy as np
import pandas as pd
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
model = osim.Model("test-output.osim")
model.finalizeConnections()
state = model.initSystem()

# Extracting the relevant markers. r_calc and L_calc. Watch out for capitalization.
# The model expected is the Plug-in-gait marker set - See vicon for more details.
right = model.getMarkerSet().get('r_calc')
left  = model.getMarkerSet().get('L_calc')

# Getting the bodies/frames we are going to connect to
exo_right = model.getBodySet().get('exo_R_feet')
exo_left = model.getBodySet().get('exo_L_feet')

"""
    SECOND PART:
    CREATING VIRTUAL MARKERS
"""
# Calculate the offset between the frames and the markers on the resting state
right_offset = right.getLocationInGround(state).to_numpy()-exo_right.getTransformInGround(state).p().to_numpy()
left_offset = left.getLocationInGround(state).to_numpy()-exo_left.getTransformInGround(state).p().to_numpy()

# Create virtual markers
marker = osim.Marker("marker_exo_R_feet",
                     exo_right,
                     osim.Vec3.createFromMat(right_offset))

model.addMarker(marker)

marker = osim.Marker("marker_exo_L_feet",
                     exo_left,
                     osim.Vec3.createFromMat(left_offset))

model.addMarker(marker)
model.finalizeConnections()
model.printToXML('test-output_virtual-markers.osim')

"""
    THIRD PART:
    ADDING THE VIRTUAL MARKER COORDINATES TO THE .TRC FILE
"""

trc_path = "walking1.trc"
output_path = "walking1_with_virtual.trc"

#Reading the file
with open(trc_path, 'r') as f:
    header = [next(f) for _ in range(4)]  # usually 6 lines
df = pd.read_csv(trc_path, sep='\t', skiprows=4)

#Find the desired marker position
cols = [c.strip() for c in header[3].split('\t') if c.strip()]
cols = cols[2:] #Skipping grame and time columns
position_r_calc = cols.index('r_calc') + 1
position_L_calc = cols.index('L_calc') + 1
total_amount = len(cols)
# Define offsets from real markers in meters (convert to mm if your .trc is in mm)
# In this case, there is no offset
offset = np.array([0.0, 0.0, 0.0])

print(df)
#Compute virtual marker coordinates - Mind the names and capitalization
df[str('X'+ total_amount+1)] = df["X"+position_r_calc] + offset[0]
print(df)
df["marker_exo_R_feet:Y"] = df["r_calc:Y"] + offset[1]
df["marker_exo_R_feet:Z"] = df["r_calc:Z"] + offset[2]

df["marker_exo_L_feet:X"] = df["L_calc:X"] + offset[0]
df["marker_exo_L_feet:Y"] = df["L_calc:Y"] + offset[1]
df["marker_exo_L_feet:Z"] = df["L_calc:Z"] + offset[2]

#Update header NumMarkers (add +N of virtual markers)
n=2
header[2] = header[2].replace(
    header[2].split()[3],
    str(int(header[2].split()[3]) + n)
)

# Name the columns
cols = list(df.columns)
cols.insert(2, "marker_exo_R_feet:X")
cols.insert(3, "marker_exo_R_feet:Y")
cols.insert(4, "marker_exo_R_feet:Z")
cols.insert(5, "marker_exo_L_feet:X")
cols.insert(6, "marker_exo_L_feet:Y")
cols.insert(7, "marker_exo_L_feet:Z")
df = df[cols]

#Save back
with open(output_path, "w") as f:
    for line in header:
        f.write(line)
    df.to_csv(f, sep='\t', index=False)